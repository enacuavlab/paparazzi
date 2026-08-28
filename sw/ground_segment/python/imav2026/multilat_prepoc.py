#!/usr/bin/env python3
import sys
from os import path, getenv, makedirs
import lxml.etree as ET
import numpy as np

PAPARAZZI_SRC  = getenv("PAPARAZZI_HOME", "/home/fabien/paparazzi")
PAPARAZZI_HOME  = getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)

if PAPARAZZI_SRC is None:
    print("PAPARAZZI_SRC not defined!")
    exit(-1)

superpysion = path.join(PAPARAZZI_SRC, "sw", "supervision", "python")
sys.path.append(superpysion)

import conf


def parse_list(txt: str):
    if txt[0] == '{':
        a = eval(f"[{txt[1:-1]}]")
        return a
    return []

def make_matrices(anchors_pos):
    xs = np.array(anchors_pos["x"])
    ys = np.array(anchors_pos["y"])
    zs = np.array(anchors_pos["z"])
    A = np.array([np.ones(xs.shape),-2*xs, -2*ys, -2*zs]).transpose()
    L = np.linalg.inv(A.transpose() @ A) @ A.transpose()
    B_c = xs**2 + ys **2 + zs **2
    return L, B_c

def generate_c_file(L, B_c, ac_name):
    dir = path.join(PAPARAZZI_HOME, "var", "aircrafts", ac_name, "ap", "generated")
    if not path.exists(dir):
        makedirs(dir)

    file_path = path.join(dir, "uwb.c")
    with open(file_path, 'w') as fout:
        fout.write("static float L[4][UWB_POSITIONING_NB_ANCHORS] = {\n")
        for row in L:
            fout.write("  {")
            for e in row:
                fout.write(f"{e}, ")
            fout.write("},\n")
        fout.write("};\n\n")

        fout.write("static const float B_c[UWB_POSITIONING_NB_ANCHORS][1] = {")
        for e in B_c:
            fout.write(f"{{{e}}}, ")
        fout.write("};\n\n")






def main(ac_name):
    c = conf.Conf("conf.xml")
    if ac_name not in c:
        print(f"{ac_name} not in conf!")
        exit(-1)
    ac_conf = c[ac_name]

    airframe_file = path.join(PAPARAZZI_HOME, "conf", ac_conf.airframe)
    airframe_xml = ET.parse(airframe_file)
    anchors_pos = {'x': None, 'y': None, 'z': None}
    for xml_section in airframe_xml.getroot().findall("section"):
        name = xml_section.get("name")
        if name == "UWB_POSITIONING":
            for define in xml_section.findall("define"):
                if define.get("name") == "ANCHORS_POS_X":
                    anchors_pos["x"] = parse_list(define.get('value'))
                if define.get("name") == "ANCHORS_POS_Y":
                    anchors_pos["y"] = parse_list(define.get('value'))
                if define.get("name") == "ANCHORS_POS_Z":
                    anchors_pos["z"] = parse_list(define.get('value'))
    if anchors_pos["x"] is None or anchors_pos["y"] is None or anchors_pos["z"] is None:
        print("ANCHOR_POS_* missing")
        exit(-1)
    L, B_c = make_matrices(anchors_pos)

    generate_c_file(L, B_c, ac_name)

    print(L)
    print(B_c)

if __name__ == "__main__":
    main(sys.argv[1])
