#!/usr/bin/env python3
import sys, time, wave
import serial

DEFAULT_SECONDS = 5.0
DEFAULT_BAUD    = 460800
DEFAULT_CHUNK   = 4096

def capture(port: str, seconds: float, baud: int, chunk: int) -> bytes:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0,          # non-blocking
        write_timeout=0,
        rtscts=False,
        dsrdtr=False,
    )

    # Flush stale bytes
    ser.reset_input_buffer()
    time.sleep(0.15)
    ser.reset_input_buffer()

    buf = bytearray()
    t0 = time.time()
    t_end = t0 + seconds

    # Read for exactly `seconds` (wall-clock)
    while True:
        now = time.time()
        if now >= t_end:
            break
        data = ser.read(chunk)
        if data:
            buf += data
        else:
            # tiny sleep so we don't busy-spin when there's nothing
            time.sleep(0.001)

    ser.close()

    # Make length even (int16)
    if len(buf) % 2 == 1:
        buf = buf[:-1]

    return bytes(buf)

def write_wav(path: str, pcm_bytes: bytes, sample_rate: int):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)          # int16
        w.setframerate(sample_rate)
        w.writeframes(pcm_bytes)

def main():
    if len(sys.argv) < 2:
        print("usage: python3 record_serial_wav.py /dev/cu.usbmodemXXXX [out.wav] [seconds] [baud]")
        sys.exit(1)

    port    = sys.argv[1]
    out     = sys.argv[2] if len(sys.argv) >= 3 else "capture.wav"
    seconds = float(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_SECONDS
    baud    = int(sys.argv[4]) if len(sys.argv) >= 5 else DEFAULT_BAUD

    print(f"[+] Port: {port}")
    print(f"[+] Baud: {baud}")
    print(f"[+] Recording: {seconds:.3f}s (wall-clock)")

    t0 = time.time()
    pcm = capture(port, seconds, baud, DEFAULT_CHUNK)
    dt = time.time() - t0

    bytes_got = len(pcm)
    samples_got = bytes_got // 2

    # Effective sample rate based on what you *actually* received
    eff_sr = int(round(samples_got / seconds)) if seconds > 0 else 16000

    print(f"[+] Got {bytes_got} bytes = {samples_got} samples")
    print(f"[+] Effective sample rate ≈ {eff_sr} Hz")
    print(f"[+] (capture loop ran {dt:.3f}s)")

    write_wav(out, pcm, eff_sr)
    print(f"[+] Wrote {out}")

if __name__ == "__main__":
    main()
