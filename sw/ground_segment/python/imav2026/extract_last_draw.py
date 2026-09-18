#!/usr/bin/env python3
"""Print lat, lon (decimal degrees), and text for the last three DRAW messages."""

import argparse
from collections import deque
import csv
import sys


def extract_last_draw(log):
    """Return up to three complete DRAW lines, in their original order."""
    messages = deque(maxlen=3)
    for line in log:
        fields = line.split(maxsplit=3)
        if len(fields) >= 3 and fields[2] == "DRAW":
            messages.append(line)
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logfile", help="Path to the Paparazzi .data log")
    args = parser.parse_args()

    try:
        with open(args.logfile, encoding="utf-8") as log:
            messages = extract_last_draw(log)
    except (OSError, UnicodeError) as error:
        parser.error(str(error))

    rows = []
    for message in messages:
        fields = message.rstrip("\r\n").split(maxsplit=10)
        if len(fields) < 10:
            parser.error("Incomplete DRAW message: " + message.rstrip())
        try:
            lat = ",".join(f"{int(value) / 1e7:.7f}" for value in fields[8].split(","))
            lon = ",".join(f"{int(value) / 1e7:.7f}" for value in fields[9].split(","))
        except ValueError:
            parser.error("Invalid DRAW coordinates: " + message.rstrip())
        rows.append((lat, lon, fields[10] if len(fields) > 10 else ""))

    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(("lat", "lon", "text"))
    writer.writerows(rows)


if __name__ == "__main__":
    main()
