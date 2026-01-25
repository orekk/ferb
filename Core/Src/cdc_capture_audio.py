#!/usr/bin/env python3
"""
Capture raw int16 mono PCM from STM32 USB CDC on macOS, print loudness stats,
(optionally) normalize, and write WAV.

Examples:
  # basic capture, write wav at assumed 16k
  python3 cdc_capture_audio.py --port /dev/cu.usbmodem2062378142341 --seconds 5 --rate 16000

  # also write a "measured-rate" wav based on capture time (helpful if firmware rate is off)
  python3 cdc_capture_audio.py --port /dev/cu.usbmodem2062378142341 --seconds 5 --rate 16000 --write_measured_wav

  # normalize to target peak (e.g., 0.9 full-scale), write normalized wav
  python3 cdc_capture_audio.py --port /dev/cu.usbmodem2062378142341 --seconds 5 --rate 16000 --normalize --target_peak 0.90
"""

import argparse
import os
import subprocess
import sys
import time
import wave
from typing import Tuple


def macos_stty_raw(port: str, baud: int = 115200) -> None:
    """Best-effort: put tty in raw mode (CDC often ignores baud but stty wants one)."""
    try:
        subprocess.run(
            ["stty", "-f", port, "raw", "clocal", "-echo", str(baud)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"[warn] stty failed (may still work): {e}")


def wait_for_port(port: str, wait_s: float = 3.0) -> None:
    """Wait until /dev/cu.usbmodem* exists (useful when device resets / re-enumerates)."""
    t0 = time.time()
    while time.time() - t0 < wait_s:
        if os.path.exists(port):
            return
        time.sleep(0.05)
    raise FileNotFoundError(f"Port not found after {wait_s:.1f}s: {port}")


def collect_exact_bytes(
    port: str,
    total_bytes: int,
    timeout_s: float,
    baud: int,
    chunk: int = 4096,
    port_wait_s: float = 3.0,
) -> Tuple[bytes, float]:
    """
    Read until total_bytes collected.
    Returns (data, elapsed_seconds).
    Times out if no progress for timeout_s.
    """
    wait_for_port(port, wait_s=port_wait_s)
    macos_stty_raw(port, baud)

    buf = bytearray()
    last_progress = time.time()
    start = time.time()

    fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK)
    try:
        while len(buf) < total_bytes:
            try:
                data = os.read(fd, min(chunk, total_bytes - len(buf)))
            except BlockingIOError:
                data = b""

            if data:
                buf.extend(data)
                last_progress = time.time()
            else:
                time.sleep(0.002)

            if time.time() - last_progress > timeout_s:
                raise TimeoutError(
                    f"No incoming USB data for {timeout_s:.1f}s. "
                    f"Collected {len(buf)}/{total_bytes} bytes."
                )

            # progress about ~once per second of audio
            if len(buf) % (16000 * 2) < 4096:
                pct = 100.0 * len(buf) / total_bytes
                sys.stdout.write(f"\rCollecting: {len(buf)}/{total_bytes} bytes ({pct:.1f}%)")
                sys.stdout.flush()

        sys.stdout.write("\nDone collecting.\n")
        sys.stdout.flush()
        elapsed = time.time() - start
        return bytes(buf), elapsed
    finally:
        os.close(fd)


def bytes_to_i16_list(raw_bytes: bytes):
    """Little-endian int16 decode without numpy."""
    n = len(raw_bytes) // 2
    out = []
    for i in range(n):
        lo = raw_bytes[2 * i]
        hi = raw_bytes[2 * i + 1]
        v = (hi << 8) | lo
        if v >= 0x8000:
            v -= 0x10000
        out.append(v)
    return out


def stats_i16(samples) -> Tuple[int, float, float]:
    """Return (peak_abs, rms, dc_mean)."""
    if not samples:
        return 0, 0.0, 0.0
    peak = 0
    ssum = 0
    sqsum = 0.0
    for v in samples:
        av = -v if v < 0 else v
        if av > peak:
            peak = av
        ssum += v
        sqsum += float(v) * float(v)
    mean = ssum / float(len(samples))
    rms = (sqsum / float(len(samples))) ** 0.5
    return peak, rms, mean


def normalize_i16(samples, target_peak_ratio: float) -> Tuple[list, float]:
    """
    Scale samples so peak ~= target_peak_ratio * 32767.
    Returns (scaled_samples, gain).
    """
    peak, _, _ = stats_i16(samples)
    if peak <= 0:
        return samples, 1.0
    target = max(1.0, min(1.0, target_peak_ratio)) * 32767.0
    gain = target / float(peak)

    out = []
    for v in samples:
        x = int(round(v * gain))
        if x > 32767:
            x = 32767
        elif x < -32768:
            x = -32768
        out.append(x)
    return out, gain


def i16_list_to_bytes(samples) -> bytes:
    """Encode signed int16 list to little-endian bytes."""
    b = bytearray()
    for v in samples:
        if v < 0:
            v += 0x10000
        b.append(v & 0xFF)
        b.append((v >> 8) & 0xFF)
    return bytes(b)


def write_wav(path: str, raw_i16le_bytes: bytes, sample_rate: int) -> None:
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # int16
        wf.setframerate(sample_rate)
        wf.writeframes(raw_i16le_bytes)


def main():
    ap = argparse.ArgumentParser(description="Capture STM32 USB CDC int16 PCM, print loudness stats, optional normalize, write WAV.")
    ap.add_argument("--port", required=True, help="e.g. /dev/cu.usbmodem2062378142341")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--rate", type=int, default=16000, help="Assumed sample rate for WAV header")
    ap.add_argument("--raw", default="audio.raw")
    ap.add_argument("--wav", default="out_assumed.wav")
    ap.add_argument("--timeout", type=float, default=3.0, help="Timeout if no data arrives")
    ap.add_argument("--baud", type=int, default=115200, help="For stty only (CDC ignores it)")
    ap.add_argument("--port_wait", type=float, default=3.0, help="Wait up to this long for the port to appear")
    ap.add_argument("--write_measured_wav", action="store_true", help="Also write out_measured.wav using measured capture rate")
    ap.add_argument("--normalize", action="store_true", help="Normalize audio to target peak and write out_normalized.wav")
    ap.add_argument("--target_peak", type=float, default=0.90, help="Peak target as ratio of full-scale (0..1). Used with --normalize")
    args = ap.parse_args()

    total_bytes = int(args.seconds * args.rate * 2)  # mono int16
    print(f"Port: {args.port}")
    print(f"Target bytes: {total_bytes}  ({args.seconds}s @ {args.rate} Hz, int16 mono)")

    raw, elapsed = collect_exact_bytes(
        args.port,
        total_bytes=total_bytes,
        timeout_s=args.timeout,
        baud=args.baud,
        port_wait_s=args.port_wait,
    )

    with open(args.raw, "wb") as f:
        f.write(raw)
    print(f"Wrote raw: {args.raw} ({len(raw)} bytes)")

    # Stats
    samples = bytes_to_i16_list(raw)
    peak, rms, mean = stats_i16(samples)
    measured_rate = int(round((len(samples) / elapsed))) if elapsed > 0 else args.rate

    print("\n--- Loudness stats (raw) ---")
    print(f"Elapsed capture time:  {elapsed:.3f} s")
    print(f"Samples captured:      {len(samples)}")
    print(f"Measured sample rate:  {measured_rate} Hz")
    print(f"Peak abs:              {peak}  (full-scale 32767)")
    print(f"RMS:                   {rms:.1f}")
    print(f"DC mean:               {mean:.1f}")

    # Write assumed-rate wav
    write_wav(args.wav, raw, args.rate)
    print(f"\nWrote WAV assumed:     {args.wav} (rate={args.rate})")

    # Write measured-rate wav
    if args.write_measured_wav:
        outm = "out_measured.wav"
        write_wav(outm, raw, measured_rate)
        print(f"Wrote WAV measured:    {outm} (rate={measured_rate})")

    # Normalize + write
    if args.normalize:
        scaled, gain = normalize_i16(samples, args.target_peak)
        norm_bytes = i16_list_to_bytes(scaled)
        outn = "out_normalized.wav"
        write_wav(outn, norm_bytes, args.rate)
        print("\n--- Normalization ---")
        print(f"Applied gain:          {gain:.2f}x to reach target_peak={args.target_peak}")
        p2, r2, m2 = stats_i16(scaled)
        print(f"New peak abs:          {p2}")
        print(f"New RMS:               {r2:.1f}")
        print(f"New DC mean:           {m2:.1f}")
        print(f"Wrote WAV normalized:  {outn} (rate={args.rate})")


if __name__ == "__main__":
    main()
