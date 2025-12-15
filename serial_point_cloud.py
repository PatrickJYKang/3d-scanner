import argparse
import math
import sys
import time

try:
    import serial  # type: ignore
except Exception:
    serial = None

import matplotlib.pyplot as plt


def compute_distance_from_voltage(voltage: float) -> float:
    return math.pow(voltage / 15.80314, 1.0 / -0.836479)


def parse_csv_data_line(line: str):
    parts = [p.strip() for p in line.split(",")]
    if parts[0] != "DATA":
        return None

    try:
        if len(parts) >= 10:
            m1_us = int(float(parts[6]))
            m2_us = int(float(parts[7]))
            voltage = float(parts[9])
        elif len(parts) >= 9:
            m1_us = int(float(parts[5]))
            m2_us = int(float(parts[6]))
            voltage = float(parts[8])
        else:
            return None
    except Exception:
        return None

    return m1_us, m2_us, voltage


def try_parse_float(line: str):
    try:
        return float(line.strip())
    except Exception:
        return None


def iter_lines_from_serial(port: str, baud: int):
    if serial is None:
        raise RuntimeError("pyserial is required. Install with: pip install pyserial")

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(1.5)

    while True:
        raw = ser.readline()
        if not raw:
            continue
        try:
            yield raw.decode(errors="ignore").strip()
        except Exception:
            continue


def iter_lines_from_file(path: str):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            yield line.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--input-file", default="", help="Read lines from a log file instead of serial")
    ap.add_argument("--mode", choices=["auto", "csv", "distance"], default="auto")

    ap.add_argument("--m1-microsteps-per-rev", type=float, default=3200.0)
    ap.add_argument("--m2-microsteps-per-rev", type=float, default=3200.0)
    ap.add_argument("--m2-mm-per-rev", type=float, default=8.0)

    ap.add_argument("--samples-per-rev", type=float, default=100.0)

    ap.add_argument("--r-offset-cm", type=float, default=0.0)
    ap.add_argument("--invert-radius", action="store_true")

    ap.add_argument("--live", action="store_true")
    ap.add_argument("--no-live", action="store_true")
    ap.add_argument("--update-every", type=int, default=50)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--max-dist-cm", type=float, default=100.0)
    args = ap.parse_args()

    if args.input_file:
        line_iter = iter_lines_from_file(args.input_file)
        live = args.live and (not args.no_live)
    else:
        if not args.port:
            print("Missing --port (or use --input-file)", file=sys.stderr)
            return 2
        line_iter = iter_lines_from_serial(args.port, args.baud)
        live = (not args.no_live)

    xs = []
    ys = []
    zs = []

    ax = None
    if not args.no_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

    def redraw():
        if ax is None:
            return
        ax.cla()
        ax.scatter(xs, ys, zs, s=2)
        ax.scatter([0.0], [0.0], [0.0], s=40, c="tab:red")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_zlabel("z (cm)")
        ax.set_box_aspect((1, 1, 1))
        plt.draw()
        plt.pause(0.001)

    if live and (not args.no_plot):
        plt.ion()
        plt.show(block=False)
        plt.pause(0.001)

    sample_i = 0
    debug_i = 0

    try:
        for line in line_iter:
            if not line:
                continue

            if args.debug:
                debug_i += 1
                print(f"RAW: {line}")

            if args.mode in ("auto", "csv"):
                parsed = parse_csv_data_line(line)
                if parsed is not None:
                    m1_us, m2_us, voltage = parsed
                    if voltage <= 0:
                        continue

                    if voltage < 1e-6:
                        if args.debug:
                            print("WARN: voltage ~ 0, skipping")
                        continue

                    dist_cm = compute_distance_from_voltage(voltage)
                    if (not math.isfinite(dist_cm)) or (dist_cm > args.max_dist_cm):
                        if args.debug:
                            print(f"DROP: dist_cm={dist_cm} (max {args.max_dist_cm})")
                        continue
                    if args.invert_radius:
                        r_cm = args.r_offset_cm - dist_cm
                    else:
                        r_cm = dist_cm + args.r_offset_cm

                    yaw = 2.0 * math.pi * (float(m1_us) / args.m1_microsteps_per_rev)
                    pitch = 2.0 * math.pi * (float(m2_us) / args.m2_microsteps_per_rev)

                    x = r_cm * math.cos(pitch) * math.cos(yaw)
                    y = r_cm * math.cos(pitch) * math.sin(yaw)
                    z = r_cm * math.sin(pitch)

                    if args.debug:
                        print(
                            "PARSED: "
                            f"m1_us={m1_us} m2_us={m2_us} voltage={voltage:.6f} "
                            f"dist_cm={dist_cm:.3f} yaw_deg={math.degrees(yaw):.2f} pitch_deg={math.degrees(pitch):.2f} "
                            f"x={x:.3f} y={y:.3f} z={z:.3f}"
                        )

                    xs.append(x)
                    ys.append(y)
                    zs.append(z)

                    if (not args.no_plot) and live and (len(xs) % args.update_every == 0):
                        redraw()

                    continue

            if args.mode in ("auto", "distance"):
                val = try_parse_float(line)
                if val is None:
                    continue

                dist_cm = val
                if (not math.isfinite(dist_cm)) or (dist_cm > args.max_dist_cm):
                    if args.debug:
                        print(f"DROP(distance): dist_cm={dist_cm} (max {args.max_dist_cm})")
                    continue
                if args.invert_radius:
                    r_cm = args.r_offset_cm - dist_cm
                else:
                    r_cm = dist_cm + args.r_offset_cm

                theta = 2.0 * math.pi * (float(sample_i) / args.samples_per_rev)
                sample_i += 1

                xs.append(r_cm * math.cos(theta))
                ys.append(r_cm * math.sin(theta))
                zs.append(0.0)

                if args.debug:
                    print(
                        "PARSED(distance): "
                        f"dist_cm={dist_cm:.3f} theta_deg={math.degrees(theta):.2f} "
                        f"x={xs[-1]:.3f} y={ys[-1]:.3f} z={zs[-1]:.3f}"
                    )

                if (not args.no_plot) and live and (len(xs) % args.update_every == 0):
                    redraw()

    except KeyboardInterrupt:
        pass

    if args.no_plot:
        return 0

    if not xs:
        print("No points parsed.", file=sys.stderr)
        return 1

    if not live:
        redraw()
        plt.show()
    else:
        plt.ioff()
        redraw()
        try:
            plt.show()
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
