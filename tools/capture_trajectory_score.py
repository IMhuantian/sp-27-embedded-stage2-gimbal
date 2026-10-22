"""Read-only serial capture of the 18-channel firmware score page; never drives motors."""
import argparse
import csv
import datetime
import json
import math
from pathlib import Path
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="SerialPlot port, e.g. COM5; close SerialPlot first")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    try:
        import serial
    except ImportError:
        parser.error("pyserial is required in this Python environment")
    output = args.output or Path(__file__).resolve().parents[1] / "build" / "score-reports" / (
        datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    latest = None
    frames = 0
    with serial.Serial(args.port, 921600, timeout=0.1) as port, output.with_suffix(".csv").open(
            "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["host_monotonic_s"] + [f"ch{i}" for i in range(18)])
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            buffer.extend(port.read(max(1, min(port.in_waiting, 8192))))
            while len(buffer) >= 75:
                offset = buffer.find(b"\xaa\xbb\x48")
                if offset < 0:
                    del buffer[:-2]
                    break
                del buffer[:offset]
                if len(buffer) < 75:
                    break
                values = struct.unpack_from("<18f", buffer, 3)
                # No wire CRC: reject impossible score/status fields and resynchronise.
                if (not all(math.isfinite(v) for v in values) or
                        not 0 <= values[14] <= 1.0001 or not 0 <= values[15] <= 1.0001 or
                        not 0 <= values[16] <= 40.001 or
                        not (0 <= values[17] <= 63 and values[17].is_integer())):
                    del buffer[0]
                    continue
                del buffer[:75]
                writer.writerow([time.monotonic(), *values])
                frames += 1
                latest = values
    report = {"captured_at": datetime.datetime.now().astimezone().isoformat(),
              "port": args.port, "frames": frames,
              "csv": str(output.with_suffix(".csv")),
              "firmware_identity": "not verified by serial protocol"}
    if latest is None or not (int(latest[17]) & 2):
        report["status"] = "no_trajectory_samples"
        report["mean_score_out_of_40"] = None
    else:
        flags = int(latest[17])
        report.update(yaw_ratio=latest[14], pitch_ratio=latest[15],
                      mean_score_out_of_40=latest[16], flags=flags,
                      samples=int(latest[12]), current_fault=int(latest[11]),
                      running=bool(flags & 1), debug_amplitude=bool(flags & 16),
                      status="invalid_run" if flags & (4 | 8 | 32) else "provisional_run_mean")
        report["official_verdict"] = "requires official trajectory, test window and mechanical compliance"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
