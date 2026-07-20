#!/usr/bin/env python3
#
# Copyright (C) 2020 Fabien Bonneval <fabien.bonneval@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#


from lxml import etree
from typing import List, Union
import sys
from xml_utils import get_attrib, get_attrib_default
import urllib.request

from math import atan2,pi,sqrt

from typing import Optional
from dataclasses import dataclass
from pyproj import Geod

geod = Geod(ellps="WGS84")


@dataclass
class Waypoint:
    name:str
    no:int
    x:Optional[float] = None
    y:Optional[float] = None
    lat:Optional[float] = None
    lon:Optional[float] = None
    alt:Optional[float] = None
    height:Optional[float] = None
    
    def __post_init__(self):
        if self.x is not None:
            self.x = float(self.x)
        if self.y is not None:
            self.y = float(self.y)
        if self.lat is not None:
            self.lat = float(self.lat)
        if self.lon is not None:
            self.lon = float(self.lon)
        if self.alt is not None:
            self.alt = float(self.alt)
        if self.height is not None:
            self.height = float(self.height)
    

class Block:
    def __init__(self, name:str, no:int, xml):
        self.name = name
        self.no = no
        self.xml = xml


class Exc:
    def __init__(self, cond, deroute):
        self.cond = cond
        self.deroute = deroute

@dataclass
class FlightPlan:
    waypoints:List[Waypoint]
    exceptions:List[Exc]
    blocks:List[Block]
    name:str
    lat0:float
    lon0:float
    max_dist_from_home:float
    ground_alt:float
    security_height:float
    alt:float
    header:str = ""


    @staticmethod
    def parse(fp_xml: str):
        if fp_xml.startswith("file://"):
            fp_tree = etree.parse(fp_xml)    
        elif fp_xml.startswith("http://"):
            tmp_file, _ = urllib.request.urlretrieve(fp_xml)
            fp_tree = etree.parse(tmp_file)
        else:
            raise ValueError("Flight plan path must start with file:// or http://")
        fp_element = fp_tree.find("flight_plan")
        
        
        name = get_attrib(fp_element, "name")
        lat0 = get_attrib(fp_element, "lat0")
        lon0 = get_attrib(fp_element, "lon0")
        max_dist_from_home = get_attrib(fp_element, "max_dist_from_home")
        ground_alt = get_attrib(fp_element, "ground_alt")
        security_height = get_attrib(fp_element, "security_height")
        alt = get_attrib(fp_element, "alt")

        if fp_element.find("header") is not None:
            header = fp_element.find("header").text
        else:
            header = ""

        ways_elt = fp_element.find("waypoints")
        waypoints = FlightPlan.__parse_waypoints(ways_elt)
        
        for wp in waypoints:
            if wp.lat is None and wp.lon is None \
                and wp.x is not None and wp.y is not None:
                
                angle = atan2(wp.y,wp.x)
                azimut = (pi/2-angle)*180/pi
                dist = sqrt(wp.x**2 + wp.y**2)
                lon,lat,_ = geod.fwd(lon0,lat0,azimut,dist)
                wp.lat = lat
                wp.lon = lon

        blocks_elt = fp_element.find("blocks")
        blocks = FlightPlan.__parse_blocks(blocks_elt)

        excs_elt = fp_element.find("exceptions")
        exceptions = FlightPlan.__parse_exceptions(excs_elt)
        return FlightPlan(
            waypoints=waypoints,
            exceptions=exceptions,
            blocks=blocks,
            name=name,
            lat0=lat0,
            lon0=lon0,
            max_dist_from_home=max_dist_from_home,
            ground_alt=ground_alt,
            security_height=security_height,
            alt=alt,
            header=header
        )

    @staticmethod
    def __parse_waypoints(ways_elt: etree.Element):
        waypoints = []
        w_no = 1    # first waypoint number is 1.
        for way_e in ways_elt.findall("waypoint"):
            name = get_attrib(way_e, "name")
            x = get_attrib_default(way_e, "x", None, float)
            y = get_attrib_default(way_e, "y", None, float)
            lat = get_attrib_default(way_e, "lat", None)
            lon = get_attrib_default(way_e, "lon", None)
            alt = get_attrib_default(way_e, "alt", None, float)
            height = get_attrib_default(way_e, "height", None, float)
            waypoint = Waypoint(name, w_no, x, y, lat, lon, alt, height)
            waypoints.append(waypoint)
            w_no += 1
        return waypoints

    @staticmethod
    def __parse_blocks(blocks_elt):
        blocks = []
        for b_e in blocks_elt.findall("block"):
            name = get_attrib(b_e, "name")
            no = get_attrib(b_e, "no")
            block = Block(name, int(no), b_e)
            blocks.append(block)
        return blocks

    @staticmethod
    def __parse_exceptions(exs_elt):
        if exs_elt is None:
            return []
        excs = []
        for ex_e in exs_elt.findall("exception"):
            cond = get_attrib(ex_e, "cond")
            deroute = get_attrib(ex_e, "deroute")
            exc = Exc(cond, deroute)
            excs.append(exc)
        return excs

    def get_waypoint(self, key: Union[str, int]):
        """
        :param key: Waypoint name or number
        :type key: str or int
        """
        if type(key) == str:
            for wp in self.waypoints:
                if wp.name == key:
                    return wp
        elif type(key) == int:
            for wp in self.waypoints:
                if wp.no == key:
                    return wp

    def get_block(self, key: Union[str, int]):
        """
        :param key: Block name or number
        :type key: str or int
        """
        if type(key) == str:
            for block in self.blocks:
                if block.name == key:
                    return block
        elif type(key) == int:
            for block in self.blocks:
                if block.no == key:
                    return block

    def get_block_groups(self):
        return list(set(filter(lambda x: x is not None,
                               [get_attrib_default(block.xml, "group", None) for block in self.blocks])))

    def get_blocks_from_group(self, groupname):
        return list(filter(lambda block: get_attrib_default(block.xml, "group", None) == groupname, self.blocks))



if __name__ == "__main__":
    flight_plan = FlightPlan.parse(sys.argv[1])
    for b in flight_plan.blocks:
        print(b.name)
