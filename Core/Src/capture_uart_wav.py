#!/usr/bin/env python3
import sys, time, wave, glob
import serial

BAUD = 460800
MAGIC = b"\xAD\xDE\xAD\xBE"
DEFAULT_OUT = "capture.wav"

def pick_default_port():
    # macOS typical ports
    cands = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/cu.usbserial*"))
    return cands[0] if cands else None

def read_exact(ser, n, overall_timeout_s=15.0):
    """Read exactly n bytes or raise TimeoutError."""
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n:
        if time.time() - t0 > overall_timeout_s:
            raise TimeoutError(f"Timeout: got {len(buf)}/{n} bytes")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
        else:
            # no data this tick; keep looping until overall timeout
            pass
    return bytes(buf)

def sync_to_magic(ser, overall_timeout_s=15.0):
    """Scan incoming stream until MAGIC appears; return any bytes after MAGIC already read."""
    window = bytearray()
    t0 = time.time()
    while True:
        if time.time() - t0 > overall_timeout_s:
            raise TimeoutError("Timeout waiting for MAGIC")
        b = ser.read(1)
        if not b:
            continue
        window += b
        if len(window) > len(MAGIC):
            window = window[-len(MAGIC):]
        if window == MAGIC:
            return  # synced

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else pick_default_port()
    out = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_OUT
    if not port:
        print("No serial ports found. Try: ls /dev/cu.usbmodem* /dev/cu.usbserial*", file=sys.stderr)
        sys.exit(1)

    print(f"[+] Opening {port} @ {BAUD}...")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    # DO NOT toggle DTR/RTS; it can reset and even change the port name on mac
    ser.reset_input_buffer()

    print("[+] Waiting for MAGIC frame (AD DE AD BE)...")
    sync_to_magic(ser, overall_timeout_s=30.0)

    print("[+] MAGIC found. Reading length...")
    length_be = read_exact(ser, 4, overall_timeout_s=5.0)
    length = int.from_bytes(length_be, "big")
    print(f"[+] Length = {length} bytes")

    print("[+] Reading payload...")
    payload = read_exact(ser, length, overall_timeout_s=30.0)
    ser.close()

    if length % 2 != 0:
        print("[!] Warning: payload length not even, WAV int16 may be wrong.", file=sys.stderr)

    # Write WAV (16-bit mono 16k)
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(payload)

    print(f"[+] Wrote {out} ({len(payload)} bytes)")

if __name__ == "__main__":
    main()