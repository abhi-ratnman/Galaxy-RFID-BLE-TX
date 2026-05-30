#!/usr/bin/env python3
"""
mu60x_test.py — interactive test tool for MU60x-series UHF RFID modules.

Connects to the module via a USB-TTL adapter on a COM port and lets you send
the most common commands from the "MU60x Host Interface Packet Definitions"
manual (v1.2.1). Defaults: 115200 8N1, address 0x00.

Frame layout:
    A0 | Len | Addr | Cmd | Data... | LRC
    Len  = bytes between Addr and LRC (i.e. 3 for no data, 4 for 1 data byte)
    LRC  = (0x100 - sum(A0..last_data)) & 0xFF
           (the leading 0xA0 is included in the checksum)

Run:
    pip install pyserial
    python mu60x_test.py --port COM5            # interactive menu
    python mu60x_test.py --port COM5 --cmd ver  # one-shot

Useful one-shots:
    ver        Get firmware version (0x72)
    power      Get current RF output power (0x77)
    setpwr N   Set RF output power N dBm, 0..33 (0x76)
    inv [ant]  Single-shot inventory, antenna 1..4 (0x80)
    rt  [ant]  Real-time inventory, prints tags as they stream (0x89)
    stop       Stop inventory (0x8C)
    reset      Reset reader (0x70)
    raw HEX... Send a raw hex frame WITHOUT the leading A0/len/lrc (we add them)
"""
import argparse
import sys
import time
import threading

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial not installed.  Run:  pip install pyserial\n")
    sys.exit(1)

HEAD = 0xA0
DEFAULT_ADDR = 0x00


# ---------------------------------------------------------------- framing ----
def lrc(buf: bytes) -> int:
    """Manual section 2.1 — (0x100 - sum) & 0xFF, computed over EVERY byte
    in the frame except the checksum itself (so the leading 0xA0 IS included).
    """
    return (0x100 - (sum(buf) & 0xFF)) & 0xFF


def build_frame(cmd: int, data: bytes = b"", addr: int = DEFAULT_ADDR) -> bytes:
    """Wrap a command + data into a full A0..LRC frame."""
    body = bytes([HEAD, 3 + len(data), addr, cmd]) + data
    return body + bytes([lrc(body)])


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# --------------------------------------------------------- read one frame ----
def read_frame(ser: serial.Serial, timeout: float = 1.0):
    """
    Read a single A0-framed packet. Returns the full frame bytes, or None on
    timeout. Resyncs to the next 0xA0 if there's garbage in the buffer.
    """
    end = time.time() + timeout
    # find 0xA0
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == HEAD:
            break
    else:
        return None
    # read Len
    ser.timeout = max(0.05, end - time.time())
    ln = ser.read(1)
    if not ln:
        return None
    rest = ser.read(ln[0])  # Addr + Cmd + Data + LRC
    if len(rest) != ln[0]:
        return None
    frame = bytes([HEAD, ln[0]]) + rest
    return frame


def parse_frame(frame: bytes):
    """Return (addr, cmd, data) for a frame; verifies the LRC."""
    if len(frame) < 5 or frame[0] != HEAD:
        return None
    body = frame[:-1]   # everything except the LRC byte
    if lrc(body) != frame[-1]:
        return ("badlrc", frame[3], frame[4:-1])
    addr = frame[2]
    cmd = frame[3]
    data = frame[4:-1]
    return (addr, cmd, data)


# ------------------------------------------------------------- transport ----
def send(ser: serial.Serial, cmd: int, data: bytes = b"", addr: int = DEFAULT_ADDR):
    frame = build_frame(cmd, data, addr)
    print(f"TX  ({len(frame):2d}): {hexs(frame)}")
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()


def recv_print(ser: serial.Serial, n_frames: int = 1, timeout: float = 1.0, stream=False):
    """Read up to n_frames frames (or stream until timeout if stream=True)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        f = read_frame(ser, timeout=max(0.1, deadline - time.time()))
        if not f:
            break
        print(f"RX  ({len(f):2d}): {hexs(f)}")
        parsed = parse_frame(f)
        if parsed:
            addr, cmd, data = parsed
            if addr == "badlrc":
                print("    ! LRC mismatch")
            else:
                print(f"    cmd=0x{cmd:02X}  data={hexs(data) or '(empty)'}")
        if not stream:
            n_frames -= 1
            if n_frames <= 0:
                return
            # for streaming-multiple, reset deadline window after a hit
            deadline = time.time() + timeout


# ---------------------------------------------------------- high-level ops ----
def cmd_version(ser):
    send(ser, 0x72)
    recv_print(ser)


def cmd_get_power(ser):
    send(ser, 0x77)
    recv_print(ser)


def cmd_set_power(ser, dbm: int):
    if not 0 <= dbm <= 33:
        print("power must be 0..33"); return
    send(ser, 0x76, bytes([dbm]))
    recv_print(ser)


def cmd_inventory(ser, ant: int = 1):
    send(ser, 0x80, bytes([ant]))
    # status frame, then a tag-count frame, then tag-data frames may follow
    recv_print(ser, n_frames=10, timeout=2.0)


def cmd_realtime(ser, ant: int = 1, seconds: float = 5.0):
    send(ser, 0x89, bytes([ant]))
    print(f"[streaming for {seconds}s — Ctrl-C to stop early]")
    end = time.time() + seconds
    try:
        while time.time() < end:
            f = read_frame(ser, timeout=min(0.5, end - time.time()))
            if not f:
                continue
            print(f"RX  ({len(f):2d}): {hexs(f)}")
            p = parse_frame(f)
            if p and p[0] != "badlrc" and p[1] == 0x89 and len(p[2]) > 5:
                ant_n = p[2][0]
                pc = p[2][1:3]
                # remaining = EPC (var) + RSSI(4) + Freq(3); split from the tail
                tail = p[2][3:]
                if len(tail) >= 7:
                    rssi = tail[-7:-3]
                    freq = tail[-3:]
                    epc = tail[:-7]
                    print(f"    ant={ant_n}  PC={hexs(pc)}  EPC={hexs(epc)}  "
                          f"RSSI={hexs(rssi)}  Freq={hexs(freq)}")
    except KeyboardInterrupt:
        pass
    finally:
        cmd_stop(ser)


def cmd_stop(ser):
    send(ser, 0x8C)
    recv_print(ser, timeout=0.5)


def cmd_reset(ser):
    send(ser, 0x70)
    recv_print(ser, timeout=2.0)


def cmd_get_select(ser):
    """Query the persistent SELECT mask (cmd 0x8E).
    A reply showing Enable=0x00 means no filter; Enable=0x01 means a
    mask is active and will hide non-matching tags from inventory."""
    send(ser, 0x8E)
    recv_print(ser, timeout=1.0)


def cmd_clear_select(ser):
    """Disable the persistent SELECT mask (cmd 0x8D, Enable=0).
    Sends a minimal disable: Enable=0, SelParam=0, Pointer=0, MaskLen=0,
    Truncate=0, no mask bytes."""
    send(ser, 0x8D, bytes([0, 0, 0, 0, 0, 0, 0, 0]))
    recv_print(ser, timeout=1.0)


def cmd_get_session(ser):
    """Get current Session and Target (cmd 0x5A).
    Reply data: <session 0..3> <target 0=A,1=B>. If target=B and tags
    are S2/S3 persistent, they won't re-answer until their flags decay."""
    send(ser, 0x5A)
    recv_print(ser, timeout=1.0)


def cmd_get_freq(ser):
    """Get current operating frequency region (cmd 0x79).
    Reply tells us which band the reader is transmitting on. If the tag
    is tuned for a different region (e.g. FCC vs EU), it won't respond."""
    send(ser, 0x79)
    recv_print(ser, timeout=1.0)


def cmd_set_region(ser, region: str):
    """Switch the operating frequency region (cmd 0x78) using factory ranges.
    region in {fcc, etsi, chn1, chn2}.  Per the manual:
        FCC   : 0x01 0x07 0x3B   (902.00 – 928.00 MHz)
        ETSI  : 0x02 0x00 0x06   (865.00 – 868.00 MHz)
        CHN_1 : 0x03 0x00 0x06   (840.00 – 845.00 MHz)
        CHN_2 : 0x05 0x2B 0x35   (920.00 – 925.00 MHz)
    Pick the one that matches the country your tags were made for. Mismatches
    are the most common reason a healthy-looking reader sees nothing."""
    regions = {
        "fcc":  bytes([0x01, 0x07, 0x3B]),
        "etsi": bytes([0x02, 0x00, 0x06]),
        "chn1": bytes([0x03, 0x00, 0x06]),
        "chn2": bytes([0x05, 0x2B, 0x35]),
    }
    key = region.lower()
    if key not in regions:
        print(f"region must be one of: {', '.join(regions)}"); return
    send(ser, 0x78, regions[key])
    recv_print(ser, timeout=1.0)


def cmd_get_link(ser):
    """Get current RF link profile (cmd 0x6A) — modulation/data rate."""
    send(ser, 0x6A)
    recv_print(ser, timeout=1.0)


def cmd_diag(ser):
    """One-shot diagnostic dump: version, power, select, session, frequency,
    link profile. Use this to compare a 'working' module against a 'broken'
    one, or to confirm settings after a factory reset."""
    print("--- ver ---");      cmd_version(ser)
    print("--- power ---");    cmd_get_power(ser)
    print("--- select ---");   cmd_get_select(ser)
    print("--- session ---");  cmd_get_session(ser)
    print("--- freq ---");     cmd_get_freq(ser)
    print("--- link ---");     cmd_get_link(ser)


_BANKS = {
    "reserved": 0, "res": 0, "0": 0,
    "epc":      1, "1": 1,
    "tid":      2, "2": 2,
    "user":     3, "usr": 3, "3": 3,
}


def _parse_bank(name: str) -> int:
    if name.lower() not in _BANKS:
        raise ValueError(f"bank must be one of: reserved/epc/tid/user (got {name!r})")
    return _BANKS[name.lower()]


def _parse_hex_bytes(s: str) -> bytes:
    """Accept '12 34 AB' or '1234AB' or '0x12 0x34' — return bytes."""
    s = s.replace("0x", "").replace(",", " ").replace(" ", "")
    if len(s) % 2 != 0:
        raise ValueError("hex must have an even number of nibbles")
    return bytes.fromhex(s)


def cmd_read_tag(ser, bank: str, word_addr: int, word_count: int, pwd_hex: str = "00000000"):
    """Read tag memory (cmd 0x81).
    Args:
        bank      reserved|epc|tid|user
        word_addr starting word address (16-bit words). For USER, start at 0.
                  For EPC, the EPC itself starts at word 2 (words 0-1 are CRC+PC).
        word_count how many WORDS (16-bit) to read. 1 word = 2 bytes.
        pwd_hex    8-hex-char access password (default all zeros for unprotected tags).
    Tag should be the only one in the RF field for predictable results."""
    pwd = _parse_hex_bytes(pwd_hex)
    if len(pwd) != 4:
        print("password must be 8 hex chars (4 bytes)"); return
    membank = _parse_bank(bank)
    data = (
        bytes([membank]) +
        word_addr.to_bytes(4, "big") +
        word_count.to_bytes(2, "big") +
        pwd
    )
    send(ser, 0x81, data)
    recv_print(ser, n_frames=2, timeout=2.0)


def cmd_write_tag(ser, bank: str, word_addr: int, data_hex: str, pwd_hex: str = "00000000"):
    """Write tag memory (cmd 0x82).
    Args:
        bank      reserved|epc|tid|user (TID is normally read-only)
        word_addr starting word address (16-bit words). For USER, start at 0.
                  For EPC, write the EPC starting at word 2.
        data_hex  hex bytes to write. Must be a whole number of WORDS (even byte count).
        pwd_hex   8-hex-char access password (default all zeros).
    Safe to retry: anything you write here can be overwritten unless the bank
    has been permalocked (state 0b10), which neither this tool nor the driver
    can do."""
    pwd = _parse_hex_bytes(pwd_hex)
    if len(pwd) != 4:
        print("password must be 8 hex chars (4 bytes)"); return
    membank = _parse_bank(bank)
    payload = _parse_hex_bytes(data_hex)
    if len(payload) == 0 or len(payload) % 2 != 0:
        print("data must be a non-empty even number of hex bytes"); return
    word_count = len(payload) // 2
    data = (
        pwd +
        bytes([membank]) +
        word_addr.to_bytes(4, "big") +
        word_count.to_bytes(2, "big") +
        payload
    )
    send(ser, 0x82, data)
    recv_print(ser, n_frames=2, timeout=2.0)


def cmd_save_params(ser):
    """Persist current reader parameters across power cycles (cmd 0x4A).
    Call this after setregion etsi / setpwr 20 etc. so the settings survive
    a reboot instead of falling back to defaults."""
    send(ser, 0x4A)
    recv_print(ser, timeout=1.0)


def cmd_factory_reset(ser):
    """Restore all reader parameters to factory defaults (cmd 0x4B).
    Per the manual: no reply packet is returned. Then we reset (0x70)."""
    send(ser, 0x4B)
    time.sleep(0.5)
    send(ser, 0x70)
    recv_print(ser, timeout=2.0)


def cmd_raw(ser, hexstr: str):
    """raw '72'  ->  send cmd 0x72 with no data. 'raw 80 02' -> cmd 0x80 data 0x02."""
    parts = [int(x, 16) for x in hexstr.replace(",", " ").split()]
    if not parts:
        print("usage: raw <cmd> [data bytes...]"); return
    send(ser, parts[0], bytes(parts[1:]))
    recv_print(ser, timeout=2.0)


# ---------------------------------------------------------------- driver ----
def run_one(ser, cmdline: str):
    toks = cmdline.strip().split()
    if not toks:
        return
    c = toks[0].lower()
    try:
        if c in ("ver", "version"):    cmd_version(ser)
        elif c in ("power", "getpwr"): cmd_get_power(ser)
        elif c == "setpwr":            cmd_set_power(ser, int(toks[1]))
        elif c == "inv":               cmd_inventory(ser, int(toks[1]) if len(toks) > 1 else 1)
        elif c == "rt":                cmd_realtime(ser, int(toks[1]) if len(toks) > 1 else 1)
        elif c == "stop":              cmd_stop(ser)
        elif c == "reset":             cmd_reset(ser)
        elif c in ("getsel", "getselect"):    cmd_get_select(ser)
        elif c in ("clrsel", "clearselect"):  cmd_clear_select(ser)
        elif c in ("factory", "factoryreset"): cmd_factory_reset(ser)
        elif c in ("save", "saveparams"):      cmd_save_params(ser)
        elif c == "read":
            # read <bank> <word_addr> <word_count> [pwd_hex]
            cmd_read_tag(ser, toks[1], int(toks[2]), int(toks[3]),
                         toks[4] if len(toks) > 4 else "00000000")
        elif c == "write":
            # write <bank> <word_addr> <data_hex> [pwd_hex]
            cmd_write_tag(ser, toks[1], int(toks[2]), toks[3],
                          toks[4] if len(toks) > 4 else "00000000")
        elif c in ("getses", "getsession"):   cmd_get_session(ser)
        elif c in ("getfreq", "getregion"):   cmd_get_freq(ser)
        elif c in ("setregion", "setfreq"):   cmd_set_region(ser, toks[1])
        elif c in ("getlink",):                cmd_get_link(ser)
        elif c in ("diag", "dump"):            cmd_diag(ser)
        elif c == "raw":               cmd_raw(ser, " ".join(toks[1:]))
        elif c in ("q", "quit", "exit"): return "quit"
        elif c in ("h", "?", "help"):  print(__doc__)
        else:
            print(f"unknown: {c}  (type 'help')")
    except Exception as e:
        print(f"error: {e}")


def interactive(ser):
    print(f"connected on {ser.portstr} @ {ser.baudrate} 8N1. type 'help' or 'q'.")
    while True:
        try:
            line = input("mu60x> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if run_one(ser, line) == "quit":
            return


def main():
    ap = argparse.ArgumentParser(description="MU60x RFID test tool")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="default 115200")
    ap.add_argument("--cmd", help="run one command and exit (e.g. 'ver' or 'inv 1')")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, bytesize=8, parity="N",
                        stopbits=1, timeout=0.2)
    try:
        if args.cmd:
            run_one(ser, args.cmd)
        else:
            interactive(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
