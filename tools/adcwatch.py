#!/usr/bin/env python3
"""
adcwatch — live raw ADC counts from a LineSights LS-6C board, one line per second.

Reads the board's `status` output over USB serial and prints the raw ADC count
(0-4095) and reported watts for every channel. Nothing is written to the board
except the `status` command. Never toggles DTR/RTS (this board's reset lines
aren't wired, and we don't want a surprise reboot anyway).

Usage:
    adcwatch.py                    live table, 1 reading/second, until Ctrl-C
    adcwatch.py -i 5               every 5 seconds
    adcwatch.py -n 10              take 10 readings then exit
    adcwatch.py --csv run1.csv     also append every reading to a CSV
    adcwatch.py -p /dev/cu.usbserial-0001   pick the port explicitly

While running, type a note and press Enter (e.g. "meter 25kW") — it's stamped
into the output and the CSV on the same line as the next reading, so meter
readings and ADC counts end up paired without any guesswork.

Columns:  time  |  CT1..CT6 raw count  |  CT1..CT6 reported W  |  note
"""

import argparse, csv, glob, re, select, sys, time

import serial  # pyserial

PORT_GLOBS = ["/dev/cu.usbserial*", "/dev/cu.SLAB*", "/dev/cu.wchusb*", "/dev/ttyUSB*"]
CT_RE = re.compile(r"CT(\d):\s*([\d.]+)A\s*\|\s*([\d.]+)W\s*\|\s*(\d+)mV")


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
    s.port = path
    s.baudrate = 115200
    s.timeout = 0.2
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    s.setDTR(False)
    s.setRTS(False)
    return s


def read_status(s, wait=3.0):
    """Send `status`, collect output, return {ch: (count, watts)} or None."""
    s.reset_input_buffer()
    s.write(b"status\n")
    s.flush()
    buf = b""
    end = time.time() + wait
    while time.time() < end:
        chunk = s.read(4096)
        if chunk:
            buf += chunk
            if b"Total:" in buf:      # status block is complete
                break
    found = {}
    for ch, amps, watts, count in CT_RE.findall(buf.decode("utf-8", "replace")):
        found[int(ch)] = (int(count), float(watts))
    return found if len(found) == 6 else None


def pending_note():
    """Non-blocking read of a line typed on stdin (POSIX)."""
    if not sys.stdin.isatty():
        return ""
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.readline().strip()
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial port (default: first USB serial found)")
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="seconds between readings (default 1)")
    ap.add_argument("-n", "--count", type=int, default=0, help="stop after N readings (default: run until Ctrl-C)")
    ap.add_argument("--csv", help="append readings to this CSV file")
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        sys.exit("no USB serial port found — is the board plugged in?")

    writer = None
    if args.csv:
        f = open(args.csv, "a", newline="")
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(["time"] + [f"ct{i}_count" for i in range(1, 7)]
                            + [f"ct{i}_watts" for i in range(1, 7)] + ["note"])

    print(f"port {port}   (type a note + Enter to tag the next reading; Ctrl-C to stop)")
    print(f"{'time':8} | {'CT1':>5} {'CT2':>5} {'CT3':>5} {'CT4':>5} {'CT5':>5} {'CT6':>5} | "
          f"{'CT1 W':>8} {'CT2 W':>8} {'CT3 W':>8} {'CT4 W':>8} {'CT5 W':>8} {'CT6 W':>8} | note")

    s = None
    taken = 0
    try:
        while True:
            if s is None:
                try:
                    s = open_port(port)
                except serial.SerialException:
                    print(f"{time.strftime('%H:%M:%S')}  port not available, retrying…", flush=True)
                    time.sleep(2)
                    continue
            try:
                data = read_status(s)
            except serial.SerialException:
                print(f"{time.strftime('%H:%M:%S')}  board disconnected, waiting…", flush=True)
                s = None
                time.sleep(2)
                continue

            note = pending_note()
            ts = time.strftime("%H:%M:%S")
            if data:
                counts = [data[i][0] for i in range(1, 7)]
                watts = [data[i][1] for i in range(1, 7)]
                print(f"{ts:8} | " + " ".join(f"{c:5d}" for c in counts) + " | "
                      + " ".join(f"{w:8.1f}" for w in watts) + (f" | {note}" if note else ""), flush=True)
                if writer:
                    writer.writerow([ts] + counts + watts + [note]); f.flush()
            else:
                print(f"{ts:8}  no status reply (board busy or different firmware?)", flush=True)

            taken += 1
            if args.count and taken >= args.count:
                break
            time.sleep(max(0.0, args.interval))
    except KeyboardInterrupt:
        pass
    finally:
        if s:
            s.close()
        if writer:
            f.close()


if __name__ == "__main__":
    main()
