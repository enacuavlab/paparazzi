#!/usr/bin/env python3
"""Repair JPEG GPS tags from modern Paparazzi DC_SHOT messages.

Use good GPS positions as anchors for piecewise constant offsets between the
last number in each filename and photo_nr. Missing files/messages do not shift
the alignment. Never interpolate coordinates or guess across an offset change.
Requires ``exif`` and ``pyproj`` (available in pprzEnv).
"""

import argparse
from bisect import bisect_left
from collections import Counter
import csv
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import shutil
import statistics
import sys
import tempfile

from exif import Image
from pyproj import Geod


GEOD = Geod(ellps="WGS84")


@dataclass(frozen=True)
class Shot:
    number: int
    latitude: float
    longitude: float
    altitude: float  # hmsl, metres above mean sea level, not ellipsoid height


@dataclass
class Photo:
    path: Path
    number: int
    gps: tuple | None
    altitude: float | None
    error: str = ""


def distance(a, b):
    """WGS84 geodesic distance in metres between (latitude, longitude) pairs."""
    _, _, metres = GEOD.inv(a[1], a[0], b[1], b[0])
    return metres


def read_shots(path, aircraft=None):
    flights = {}
    with path.open() as stream:
        for lineno, line in enumerate(stream, 1):
            fields = line.split()
            if len(fields) < 3 or fields[2] != "DC_SHOT":
                continue
            if aircraft is not None and fields[1] != aircraft:
                continue
            try:
                if len(fields) != 14:
                    raise ValueError("expected WGS84 DC_SHOT (11 data fields)")
                shot = Shot(int(fields[3]), int(fields[4]) / 1e7,
                            int(fields[5]) / 1e7, int(fields[7]) / 1000)
                if not (-90 <= shot.latitude <= 90 and -180 <= shot.longitude <= 180):
                    raise ValueError("invalid log coordinates")
                shots = flights.setdefault(fields[1], {})
                if shot.number in shots:
                    if shots[shot.number] != shot:
                        raise ValueError("conflicting repeated shot number; split logs at counter resets")
                    continue  # identical telemetry retransmission
                if shots and shot.number <= max(shots):
                    raise ValueError("shot numbers are not increasing; split logs at counter resets")
                shots[shot.number] = shot
            except ValueError as exc:
                raise ValueError(f"{path}:{lineno}: {exc}") from exc
    if not flights:
        raise ValueError("no DC_SHOT messages found for the requested aircraft")
    if len(flights) != 1:
        raise ValueError(f"multiple aircraft ({', '.join(flights)}); select one with --aircraft")
    return next(iter(flights.items()))


def exif_segment(data):
    """Locate the EXIF APP1 before JPEG scan data, leaving XMP/ICC untouched."""
    if data[:2] != b"\xff\xd8":
        raise ValueError("not a JPEG")
    pos = 2
    found = None
    while pos < len(data):
        start = pos
        if data[pos] != 0xff:
            raise ValueError("invalid JPEG marker")
        while pos < len(data) and data[pos] == 0xff:
            pos += 1
        if pos >= len(data):
            break
        marker = data[pos]
        pos += 1
        if marker in (0xda, 0xd9):  # start of scan, end of image
            return found
        if marker == 0x01 or 0xd0 <= marker <= 0xd7:
            continue
        size = int.from_bytes(data[pos:pos + 2], "big")
        end = pos + size
        if size < 2 or end > len(data):
            raise ValueError("truncated JPEG segment")
        if marker == 0xe1 and data[pos + 2:pos + 8] == b"Exif\0\0":
            if found is not None:
                raise ValueError("multiple EXIF segments")
            found = (start, end)
        pos = end
    raise ValueError("JPEG has no scan/end marker")


def metadata(data):
    segment = exif_segment(data)
    # Isolate EXIF so the library cannot mistake an XMP APP1 for EXIF.
    payload = data[slice(*segment)] if segment else b""
    return Image(b"\xff\xd8" + payload + b"\xff\xd9")


def read_gps(meta):
    def degrees(tag, positive, negative, limit):
        d, m, s = map(float, meta.get(tag))
        ref = meta.get(tag + "_ref")
        value = d + m / 60 + s / 3600
        if (ref not in (positive, negative) or not all(map(math.isfinite, (d, m, s)))
                or d < 0 or not 0 <= m < 60 or not 0 <= s < 60 or value > limit):
            raise ValueError("invalid GPS coordinate")
        return -value if ref == negative else value

    try:
        gps = (degrees("gps_latitude", "N", "S", 90),
               degrees("gps_longitude", "E", "W", 180))
    except Exception:  # malformed rational values are also broken GPS
        gps = None
    try:
        altitude = float(meta.get("gps_altitude"))
        ref = int(meta.get("gps_altitude_ref"))
        if not math.isfinite(altitude) or altitude < 0 or ref not in (0, 1):
            raise ValueError("invalid GPS altitude")
        altitude *= -1 if ref else 1
    except Exception:
        altitude = None
    return gps, altitude


def read_photos(directory):
    photos = []
    for path in sorted(directory.iterdir()):
        if not path.is_file() or path.suffix.lower() not in (".jpg", ".jpeg"):
            continue
        match = re.search(r"(\d+)(?!.*\d)", path.stem)
        if match is None:
            raise ValueError(f"no image number in {path.name}")
        photo = Photo(path, int(match[1]), None, None)
        try:
            if path.is_symlink():
                raise ValueError("symbolic links are unsupported")
            photo.gps, photo.altitude = read_gps(metadata(path.read_bytes()))
        except Exception as exc:
            photo.error = str(exc)
        photos.append(photo)
    photos.sort(key=lambda p: p.number)
    if not photos:
        raise ValueError("no JPEG images found")
    if len({p.number for p in photos}) != len(photos):
        raise ValueError("duplicate image numbers; use a directory with one camera sequence")
    return photos


def find_anchors(photos, shots, radius):
    candidates = {}
    for photo in photos:
        if photo.gps is None or photo.error:
            continue
        close = [s.number for s in shots.values()
                 if distance(photo.gps, (s.latitude, s.longitude)) <= radius]
        if len(close) == 1:
            candidates[photo.number] = close[0]
    counts = Counter(shot - image for image, shot in candidates.items())
    anchors = {image: shot for image, shot in candidates.items()
               if counts[shot - image] >= 3}
    if len(anchors) < 3:
        raise ValueError("need at least three unique GPS matches supporting an offset; "
                         "use --offset only if the numbering offset is known")
    values = list(anchors.values())
    if any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError("GPS matches contradict image order; cannot align safely")
    return anchors


def aligned_number(number, anchors):
    if number in anchors:
        return anchors[number]
    numbers = sorted(anchors)
    index = bisect_left(numbers, number)
    # Require two agreeing nearby anchors, also for extrapolation at either end.
    if index == 0:
        left, right = numbers[:2]
    elif index == len(numbers):
        left, right = numbers[-2:]
    else:
        left, right = numbers[index - 1:index + 1]
    offset = anchors[left] - left
    if offset == anchors[right] - right:
        return number + offset
    return None


def make_plan(photos, shots, anchors, offset, threshold, altitude_threshold):
    rows = []
    for photo in photos:
        number = photo.number + offset if offset is not None else aligned_number(photo.number, anchors)
        shot = shots.get(number)
        row = dict(filename=photo.path.name, image_number=photo.number,
                   shot_number=number, status="", reason="", distance_m=None,
                   altitude_error_m=None, old_latitude=photo.gps[0] if photo.gps else None,
                   old_longitude=photo.gps[1] if photo.gps else None,
                   old_altitude=photo.altitude, latitude=shot.latitude if shot else None,
                   longitude=shot.longitude if shot else None, altitude=shot.altitude if shot else None,
                   fix_position=False, fix_altitude=False)
        if photo.error:
            row.update(status="skip", reason="unreadable image/EXIF: " + photo.error)
        elif shot is None:
            row.update(status="skip", reason="ambiguous alignment" if number is None
                       else "DC_SHOT missing from log")
        else:
            horizontal = distance(photo.gps, (shot.latitude, shot.longitude)) if photo.gps else None
            vertical = abs(photo.altitude - shot.altitude) if photo.altitude is not None else None
            fix_position = horizontal is None or horizontal > threshold
            fix_altitude = vertical is None or vertical > altitude_threshold
            row.update(distance_m=horizontal, altitude_error_m=vertical,
                       fix_position=fix_position, fix_altitude=fix_altitude,
                       status="repair" if fix_position or fix_altitude else "ok")
        rows.append(row)
    return rows


def dms(value):
    seconds = round(abs(value) * 3600, 7)
    degrees, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    return int(degrees), int(minutes), round(seconds, 7)


def repaired_bytes(data, row):
    meta = metadata(data)
    if row["fix_position"]:
        meta.gps_latitude = dms(row["latitude"])
        meta.gps_latitude_ref = "S" if row["latitude"] < 0 else "N"
        meta.gps_longitude = dms(row["longitude"])
        meta.gps_longitude_ref = "W" if row["longitude"] < 0 else "E"
    if row["fix_altitude"]:
        meta.gps_altitude = abs(row["altitude"])
        meta.gps_altitude_ref = 1 if row["altitude"] < 0 else 0
    encoded = meta.get_file()
    new_segment = exif_segment(encoded)
    if new_segment is None:
        raise ValueError("EXIF writer produced no EXIF segment")
    start, end = exif_segment(data) or (2, 2)
    result = data[:start] + encoded[slice(*new_segment)] + data[end:]
    gps, altitude = read_gps(metadata(result))
    if row["fix_position"] and (gps is None or distance(gps, (row["latitude"], row["longitude"])) > 0.01):
        raise ValueError("GPS verification failed after encoding")
    if row["fix_altitude"] and (altitude is None or abs(altitude - row["altitude"]) > 0.01):
        raise ValueError("altitude verification failed after encoding")
    return result


def apply_repair(path, row):
    original = path.read_bytes()
    repaired = repaired_bytes(original, row)
    # Exclusive creation never overwrites a previous original.
    backup = path.parent / "originals" / path.name
    backup.parent.mkdir(exist_ok=True)
    with backup.open("xb") as stream:
        stream.write(original)
        stream.flush()
        os.fsync(stream.fileno())
    shutil.copystat(path, backup)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".gps-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(repaired)
            stream.flush()
            os.fsync(stream.fileno())
        shutil.copystat(path, temporary)
        if path.read_bytes() != original:
            raise ValueError("image changed during repair")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="Paparazzi .data file")
    parser.add_argument("directory", type=Path, help="directory containing numbered JPEGs")
    parser.add_argument("--apply", action="store_true", help="write repairs; default is a dry run")
    parser.add_argument("--aircraft", help="aircraft ID when multiple aircraft occur in the log")
    parser.add_argument("--offset", type=int, help="known shot_number - image_number; bypass automatic alignment")
    parser.add_argument("--anchor-distance", type=positive_float, default=8, metavar="M")
    parser.add_argument("--threshold", type=positive_float, default=10, metavar="M",
                        help="repair horizontal discrepancies above this distance (default: 10 m)")
    parser.add_argument("--altitude-threshold", type=positive_float, default=10, metavar="M",
                        help="repair altitude discrepancies above this distance (default: 10 m)")
    parser.add_argument("--report", type=Path, help="write per-image analysis as CSV")
    args = parser.parse_args(argv)
    try:
        if args.anchor_distance > args.threshold:
            raise ValueError("--anchor-distance must not exceed --threshold")
        aircraft, shots = read_shots(args.log, args.aircraft)
        photos = read_photos(args.directory)
        if args.report and args.report.resolve() in {args.log.resolve(), *(p.path.resolve() for p in photos)}:
            raise ValueError("report path must not overwrite the log or an image")
        anchors = find_anchors(photos, shots, args.anchor_distance) if args.offset is None else {}
        rows = make_plan(photos, shots, anchors, args.offset, args.threshold, args.altitude_threshold)
        print(f"Aircraft {aircraft}: {len(shots)} shots, {len(photos)} images")
        if anchors:
            print(f"GPS anchors: {len(anchors)}; shot - image offsets: "
                  f"{dict(sorted(Counter(s - p for p, s in anchors.items()).items()))}")
        else:
            print(f"Explicit shot - image offset: {args.offset}")
        good = [r["distance_m"] for r in rows if r["status"] == "ok"]
        if good:
            print(f"Good GPS distances: median {statistics.median(good):.2f} m, max {max(good):.2f} m")
        # Check every destination before starting any writes.
        if args.apply:
            for photo, row in zip(photos, rows):
                if row["status"] == "repair" and (photo.path.parent / "originals" / photo.path.name).exists():
                    raise ValueError(f"backup already exists for {photo.path.name}; inspect it before retrying")
        for photo, row in zip(photos, rows):
            if row["status"] == "repair":
                error = "missing/invalid" if row["distance_m"] is None else f'{row["distance_m"]:.2f} m'
                print(f'{photo.path.name}: shot {row["shot_number"]}, GPS error {error}; '
                      f'target {row["latitude"]:.7f}, {row["longitude"]:.7f}, hmsl {row["altitude"]:.3f} m; '
                      f'fix position={row["fix_position"]}, altitude={row["fix_altitude"]}')
                if args.apply:
                    apply_repair(photo.path, row)
                    row["status"] = "repaired"
            elif row["status"] == "skip":
                print(f'{photo.path.name}: SKIP ({row["reason"]})')
        if args.report:
            with args.report.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        print(("Applied: " if args.apply else "Dry run (use --apply to write): ")
              + ", ".join(f"{count} {status}" for status, count in sorted(Counter(r["status"] for r in rows).items())))
        return 0
    except (OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
