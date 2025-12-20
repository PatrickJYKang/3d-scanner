import argparse
import math
import os
import pathlib
import shutil
import subprocess
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


def iter_lines_from_serial(port: str, baud: int, reset: bool = True):
    if serial is None:
        raise RuntimeError("pyserial is required. Install with: pip install pyserial")

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.2)

    if reset and getattr(ser, "dtr", None) is not None:
        try:
            ser.dtr = False
            time.sleep(0.2)
            ser.dtr = True
        except Exception:
            pass

        time.sleep(1.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

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


def set_axes_equal_3d(ax, xs, ys, zs):
    if not xs:
        return

    xmin = min(min(xs), 0.0)
    xmax = max(max(xs), 0.0)
    ymin = min(min(ys), 0.0)
    ymax = max(max(ys), 0.0)
    zmin = min(min(zs), 0.0)
    zmax = max(max(zs), 0.0)

    xmid = 0.5 * (xmin + xmax)
    ymid = 0.5 * (ymin + ymax)
    zmid = 0.5 * (zmin + zmax)

    xspan = xmax - xmin
    yspan = ymax - ymin
    zspan = zmax - zmin
    max_span = max(xspan, yspan, zspan)
    if max_span <= 0:
        max_span = 1.0

    half = 0.5 * max_span
    ax.set_xlim(xmid - half, xmid + half)
    ax.set_ylim(ymid - half, ymid + half)
    ax.set_zlim(zmid - half, zmid + half)


def filter_points_radius_neighbors(xs, ys, zs, radius_cm: float, min_neighbors: int, max_points: int):
    if radius_cm <= 0:
        return xs, ys, zs
    if min_neighbors <= 0:
        return xs, ys, zs
    if len(xs) != len(ys) or len(xs) != len(zs):
        return xs, ys, zs

    n = len(xs)
    if n <= min_neighbors:
        return [], [], []

    start = 0
    if max_points > 0 and n > max_points:
        start = n - max_points

    cell = radius_cm
    r2 = radius_cm * radius_cm
    buckets = {}

    def cell_index(v: float) -> int:
        return int(math.floor(v / cell))

    for i in range(start, n):
        key = (cell_index(xs[i]), cell_index(ys[i]), cell_index(zs[i]))
        if key in buckets:
            buckets[key].append(i)
        else:
            buckets[key] = [i]

    keep_x = []
    keep_y = []
    keep_z = []

    for i in range(start, n):
        cx = cell_index(xs[i])
        cy = cell_index(ys[i])
        cz = cell_index(zs[i])

        count = 0
        accepted = False

        for dx_cell in (-1, 0, 1):
            for dy_cell in (-1, 0, 1):
                for dz_cell in (-1, 0, 1):
                    key = (cx + dx_cell, cy + dy_cell, cz + dz_cell)
                    for j in buckets.get(key, []):
                        if j == i:
                            continue
                        dx = xs[i] - xs[j]
                        dy = ys[i] - ys[j]
                        dz = zs[i] - zs[j]
                        if (dx * dx + dy * dy + dz * dz) <= r2:
                            count += 1
                            if count >= min_neighbors:
                                accepted = True
                                break
                    if accepted:
                        break
                if accepted:
                    break
            if accepted:
                break

        if accepted:
            keep_x.append(xs[i])
            keep_y.append(ys[i])
            keep_z.append(zs[i])

    return keep_x, keep_y, keep_z


def apply_neighbor_filter(xs, ys, zs, radius_cm: float, min_neighbors: int, iterations: int, max_points: int):
    if iterations <= 0:
        iterations = 1

    cur_x, cur_y, cur_z = xs, ys, zs
    for _ in range(iterations):
        nxt_x, nxt_y, nxt_z = filter_points_radius_neighbors(cur_x, cur_y, cur_z, radius_cm, min_neighbors, max_points)
        if len(nxt_x) == 0 and len(cur_x) != 0:
            return cur_x, cur_y, cur_z

        cur_x, cur_y, cur_z = nxt_x, nxt_y, nxt_z

    return cur_x, cur_y, cur_z


def maybe_upload_sketch(arduino_cli: str, fqbn: str, port: str, sketch: str):
    if not fqbn:
        raise RuntimeError("Missing --fqbn (e.g. arduino:avr:uno)")
    if not port:
        raise RuntimeError("Missing --port")
    if not sketch:
        raise RuntimeError("Missing --sketch")

    if shutil.which(arduino_cli) is None:
        raise RuntimeError(f"arduino-cli not found: {arduino_cli}")

    sketch_path = pathlib.Path(sketch).expanduser()
    if sketch_path.is_dir():
        sketch_dir = sketch_path
    else:
        sketch_dir = sketch_path.parent

    subprocess.run([arduino_cli, "compile", "--fqbn", fqbn, str(sketch_dir)], check=True)
    subprocess.run([arduino_cli, "upload", "-p", port, "--fqbn", fqbn, str(sketch_dir)], check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--input-file", default="", help="Read lines from a log file instead of serial")
    ap.add_argument("--mode", choices=["auto", "csv", "distance"], default="auto")

    ap.add_argument("--upload", action="store_true")
    ap.add_argument("--arduino-cli", default="arduino-cli")
    ap.add_argument("--fqbn", default="")
    ap.add_argument(
        "--sketch",
        default=str(pathlib.Path(os.getcwd()) / "move_and_read" / "move_and_read.ino"),
    )

    ap.add_argument("--reset", dest="reset", action="store_true")
    ap.add_argument("--no-reset", dest="reset", action="store_false")
    ap.set_defaults(reset=True)

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

    ap.add_argument("--neighbor-radius-cm", type=float, default=3.0)
    ap.add_argument("--neighbor-min-neighbors", type=int, default=4)
    ap.add_argument("--neighbor-max-points", type=int, default=0)
    ap.add_argument("--neighbor-iterations", type=int, default=2)
    ap.add_argument("--neighbor-filter", dest="neighbor_filter", action="store_true")
    ap.add_argument("--no-neighbor-filter", dest="neighbor_filter", action="store_false")
    ap.set_defaults(neighbor_filter=True)

    args = ap.parse_args()

    if args.input_file:
        line_iter = iter_lines_from_file(args.input_file)
        live = args.live and (not args.no_live)
    else:
        if not args.port:
            print("Missing --port (or use --input-file)", file=sys.stderr)
            return 2

        if args.upload:
            maybe_upload_sketch(args.arduino_cli, args.fqbn, args.port, args.sketch)

        line_iter = iter_lines_from_serial(args.port, args.baud, reset=args.reset)
        live = args.live and (not args.no_live)

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

        plot_xs, plot_ys, plot_zs = xs, ys, zs
        if live and args.neighbor_filter and len(xs) > 0:
            plot_xs, plot_ys, plot_zs = apply_neighbor_filter(
                xs,
                ys,
                zs,
                radius_cm=args.neighbor_radius_cm,
                min_neighbors=args.neighbor_min_neighbors,
                iterations=args.neighbor_iterations,
                max_points=args.neighbor_max_points,
            )
            if args.debug:
                print(
                    f"FILTER(live): kept {len(plot_xs)}/{len(xs)} "
                    f"(r={args.neighbor_radius_cm}, n={args.neighbor_min_neighbors}, it={args.neighbor_iterations})"
                )

        ax.scatter(plot_xs, plot_ys, plot_zs, s=2)
        ax.scatter([0.0], [0.0], [0.0], s=40, c="tab:red")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_zlabel("z (cm)")
        ax.set_box_aspect((1, 1, 1))
        set_axes_equal_3d(ax, plot_xs, plot_ys, plot_zs)
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

            if line.upper().startswith("DONE"):
                break

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

    if (not live) and args.neighbor_filter and len(xs) > 0:
        if args.debug:
            print(
                f"FILTER(post): running radius outlier removal on {len(xs)} points "
                f"(r={args.neighbor_radius_cm}, n={args.neighbor_min_neighbors}, it={args.neighbor_iterations})"
            )
        xs, ys, zs = apply_neighbor_filter(
            xs,
            ys,
            zs,
            radius_cm=args.neighbor_radius_cm,
            min_neighbors=args.neighbor_min_neighbors,
            iterations=args.neighbor_iterations,
            max_points=0,
        )
        if args.debug:
            print(f"FILTER(post): kept {len(xs)} points")

    if args.no_plot:
        return 0

    if not xs:
        print("No points parsed.", file=sys.stderr)
        return 1

    if not live:
        redraw()
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
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
