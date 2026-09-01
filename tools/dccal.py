#!/usr/bin/env python3
"""
dccal — ADC-count -> kW calibration for a LineSights DC-tap channel.

Workflow (board running normal 2.17.4.1 firmware, NOT download mode):

    dccal.py                  # channel 1 by default; -c 5 for another channel
        live counts stream on screen. When you read the meter, type the kW
        and press Enter, e.g.   25      -> records (current count, 25 kW)
        Commands while running:
            <number>   record a point at this instant
            fit        show the fit (through-origin AND free-intercept)
            list       show recorded points
            drop N     delete point N
            apply      write the through-origin fit to the board (dcset)
            quit

Points persist in dccal_points_ch<N>.json next to this script, so you can
gather them across several furnace runs and fit once.

Why two fits: the sensor reads 0 counts at 0 kW (measured), so the physically
right model is kW = slope*count with NO offset. The free-intercept fit is shown
only as a check — a large positive intercept means the points are inconsistent
or too narrow in range, not that the furnace draws power at zero signal.
"""

import argparse, glob, json, os, re, select, sys, time
import serial

PORT_GLOBS = ["/dev/cu.usbserial*", "/dev/cu.SLAB*", "/dev/cu.wchusb*", "/dev/ttyUSB*"]
CT_RE = re.compile(r"CT(\d):\s*[\d.]+A\s*\|\s*[\d.]+W\s*\|\s*(\d+)mV")
HERE = os.path.dirname(os.path.abspath(__file__))


def find_port(explicit):
    if explicit:
        return explicit
    for g in PORT_GLOBS:
        hits = sorted(glob.glob(g))
        if hits:
            return hits[0]
    return None


def open_port(path):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = path, 115200, 0.2
    s.dsrdtr = s.rtscts = False
    s.open(); s.setDTR(False); s.setRTS(False)
    return s


def read_counts(s, wait=3.0):
    s.reset_input_buffer(); s.write(b"status\n"); s.flush()
    buf = b""; end = time.time() + wait
    while time.time() < end:
        c = s.read(4096)
        if c:
            buf += c
            if b"Total:" in buf:
                break
    found = {int(ch): int(cnt) for ch, cnt in CT_RE.findall(buf.decode("utf-8", "replace"))}
    return found if len(found) == 6 else None


def send_cmd(s, cmd, wait=3.0):
    s.reset_input_buffer(); s.write((cmd + "\n").encode()); s.flush()
    buf = b""; end = time.time() + wait
    while time.time() < end:
        buf += s.read(4096)
    return buf.decode("utf-8", "replace")


def fit_free(pts):
    n = len(pts)
    if n < 2:
        return None
    sx = sum(c for c, _ in pts); sy = sum(k for _, k in pts)
    sxx = sum(c * c for c, _ in pts); sxy = sum(c * k for c, k in pts)
    d = n * sxx - sx * sx
    if abs(d) < 1e-9:
        return None
    a = (n * sxy - sx * sy) / d; b = (sy - a * sx) / n
    my = sy / n
    sst = sum((k - my) ** 2 for _, k in pts); ssr = sum((k - (a * c + b)) ** 2 for c, k in pts)
    return a, b, (1 - ssr / sst) if sst > 1e-12 else 1.0


def fit_origin(pts):
    sxx = sum(c * c for c, _ in pts); sxy = sum(c * k for c, k in pts)
    if sxx < 1e-9:
        return None
    a = sxy / sxx
    my = sum(k for _, k in pts) / len(pts)
    sst = sum((k - my) ** 2 for _, k in pts); ssr = sum((k - a * c) ** 2 for c, k in pts)
    return a, (1 - ssr / sst) if sst > 1e-12 else 1.0


def show_fit(pts, ch):
    if len(pts) < 1:
        print("  no points yet"); return None
    print(f"\n--- CH{ch} fit over {len(pts)} point(s) ---")
    fo = fit_origin(pts)
    if fo:
        a, r2 = fo
        print(f"  THROUGH ORIGIN (use this):  kW = {a:.6f} * count          R^2 = {r2:.5f}")
        print("  count |  meter kW |  fit kW  (err)")
        for c, k in pts:
            print(f"  {c:5.0f} | {k:8.2f} | {a*c:8.2f} ({a*c-k:+.2f})")
        print(f"  -> resolution: 1 count = {a*1000:.0f} W;  +/-20 counts noise = +/-{a*20:.1f} kW")
    ff = fit_free(pts)
    if ff:
        a2, b2, r22 = ff
        print(f"  free-intercept (check only): kW = {a2:.6f}*count {b2:+.2f}   R^2 = {r22:.5f}")
        if b2 > 5:
            print(f"  ** intercept +{b2:.1f} kW: points disagree or range too narrow — get a low-load point **")
    lo = min(k for _, k in pts); hi = max(k for _, k in pts)
    if len(pts) >= 2 and hi > 0 and (hi - lo) / hi < 0.5:
        print(f"  ** points span only {lo:.0f}-{hi:.0f} kW — add points at very different loads **")
    return fo


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--channel", type=int, default=1, help="CT channel 1-6 (default 1)")
    ap.add_argument("-p", "--port")
    args = ap.parse_args()
    ch = args.channel
    if not 1 <= ch <= 6:
        sys.exit("channel must be 1-6")

    store = os.path.join(HERE, f"dccal_points_ch{ch}.json")
    pts = json.load(open(store)) if os.path.exists(store) else []
    pts = [tuple(p) for p in pts]

    port = find_port(args.port)
    if not port:
        sys.exit("no USB serial port found — is the board plugged in and running (not download mode)?")
    s = open_port(port)
    print(f"port {port} | calibrating CH{ch} | {len(pts)} saved point(s) in {os.path.basename(store)}")
    print("type meter kW + Enter to record | fit | list | drop N | apply | quit\n")

    recent = []
    try:
        while True:
            d = read_counts(s)
            if d is None:
                print(f"{time.strftime('%H:%M:%S')}  no status reply — board in download mode or not running firmware?")
                time.sleep(2); continue
            cnt = d[ch]
            recent = (recent + [cnt])[-5:]
            others = " ".join(f"CT{k}={d[k]}" for k in range(1, 7) if k != ch)
            print(f"\r{time.strftime('%H:%M:%S')}  CH{ch} count = {cnt:5d}   (last5 avg {sum(recent)/len(recent):6.1f})   [{others}]   ", end="", flush=True)

            r, _, _ = select.select([sys.stdin], [], [], 0.8)
            if not r:
                continue
            line = sys.stdin.readline().strip()
            print()
            if not line:
                continue
            if line == "quit":
                break
            elif line == "list":
                for i, (c, k) in enumerate(pts, 1):
                    print(f"  {i}: count={c:.1f}  meter={k:.2f} kW")
                if not pts: print("  (none)")
            elif line == "fit":
                show_fit(pts, ch)
            elif line.startswith("drop"):
                try:
                    i = int(line.split()[1]) - 1
                    p = pts.pop(i); json.dump(pts, open(store, "w"))
                    print(f"  dropped point {i+1}: count={p[0]:.1f} meter={p[1]:.2f}")
                except Exception:
                    print("  usage: drop N")
            elif line == "apply":
                fo = fit_origin(pts) if pts else None
                if not fo:
                    print("  nothing to apply — record points first"); continue
                a, _ = fo
                cmd = f"dcset {ch} {a:.6f} 0"
                print(f"  sending: {cmd}")
                out = send_cmd(s, cmd)
                print("  board: " + " | ".join(l for l in out.splitlines() if "[DC]" in l))
                print(f"  server equivalent: {{\"action\":\"set_dc_cal\",\"channel\":{ch},\"slope\":{a:.6f},\"offset\":0}}")
            else:
                try:
                    kw = float(line)
                except ValueError:
                    print("  ? type a kW number, or: fit | list | drop N | apply | quit"); continue
                # fresh sample RIGHT NOW so the pairing is honest
                d2 = read_counts(s)
                c_now = d2[ch] if d2 else cnt
                pts.append((float(c_now), kw)); json.dump(pts, open(store, "w"))
                print(f"  recorded point {len(pts)}: count={c_now}  meter={kw:.2f} kW")
                show_fit(pts, ch)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        print(f"\n{len(pts)} point(s) saved to {store}")


if __name__ == "__main__":
    main()
