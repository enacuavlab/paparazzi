#!/usr/bin/env python3

import argparse
import numpy as np

import json

import pyproj

from Dubins import BasicPath

def main():
    parser = argparse.ArgumentParser(description='Given a list of waypoints, generate a list of barriers (obstacles) along the path between them. Also performs coordinate transformation if needed.')
    parser.add_argument('waypoints', type=float, nargs='+', help='List of waypoints as x1 y1 x2 y2 ...')
    parser.add_argument('--close', action='store_true', help='If .')
    parser.add_argument('-o','--output', type=str, default='', help='Output JSON file for obstacles. If empty (default), output to stdout.')
    parser.add_argument('--crs-in', type=str, default='WGS84', help='Pyproj CRS code for input coordinates (default: WGS84)')
    parser.add_argument('--crs-out', type=str, default='EPSG:9794', help='Pyproj CRS code for output coordinates (default: EPSG:9794, ie. Lambert93)')
    args = parser.parse_args()

    transformer = pyproj.Transformer.from_crs(args.crs_in, args.crs_out) # Default to WGS84 to Lambert93
    
    pts = []
    for i in range(0, len(args.waypoints), 2):
        x, y = transformer.transform(args.waypoints[i], args.waypoints[i+1])
        pts.append((x, y, 0))
    
    paths = []
    for i in range(len(pts)-1):
        p1 = pts[i]
        p2 = pts[i+1]
        path = BasicPath.from_2_points(p1, p2)
        paths.append(path.asdict())
    
    if args.close:
        p1 = pts[-1]
        p2 = pts[0]
        path = BasicPath.from_2_points(p1, p2)
        paths.append(path.asdict())
    
    output = {"sections": paths}
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(output, f, indent=4)
    else:
        print(json.dumps(output, indent=4))
        
if __name__ == "__main__":
    main()
    
        

