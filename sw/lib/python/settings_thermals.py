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
from typing import List
import sys
from os import path, getenv
import time
from xml_utils import get_attrib, get_attrib_default

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import matplotlib.animation as an
from mpl_toolkits.mplot3d import Axes3D


class PprzSettingsManager4:
    def __init__(self, ac_id, ivy: IvyMessagesInterface):
        self.ac_id = ac_id
        self.ivy = ivy
        self.diest = {}
        self.ivy.subscribe(self.get_z_dot, PprzMessage('ground', 'WIND'))


    def get_z_dot(self, sender, msg):
        self.diest['var'] = 'wind_speed'
        self.diest['shortname'] = 'wind_speed'
        self.diest['value'] = msg.mean_aspeed


class PprzSettingsManager3:
    def __init__(self, ac_id, ivy: IvyMessagesInterface):
        self.ac_id = ac_id
        self.ivy = ivy
        self.dix = {}
        self.diy = {}
        self.ivy.subscribe(self.get_x, PprzMessage('telemetry', 'NAVIGATION'))
        self.ivy.subscribe(self.get_y, PprzMessage('telemetry', 'NAVIGATION'))


    def get_x(self, sender, msg):
        self.dix['var'] = 'positionX'
        self.dix['shortname'] = 'positionX'
        self.dix['value'] = msg.pos_x
        
    def get_y(self, sender, msg):
        self.diy['var'] = 'positionY'
        self.diy['shortname'] = 'positionY'
        self.diy['value'] = msg.pos_y



class PprzSettingsManager2:
    def __init__(self, ac_id, ivy: IvyMessagesInterface):
        self.ac_id = ac_id
        self.ivy = ivy
        self.diair = {}
        self.dialt = {}
        self.ivy.subscribe(self.get_airspeed, PprzMessage('ground', 'FLIGHT_PARAM'))
        self.ivy.subscribe(self.get_altitude, PprzMessage('ground', 'FLIGHT_PARAM'))


    def get_airspeed(self, sender, msg):
        self.diair['var'] = 'airspeed'
        self.diair['shortname'] = 'airspeed'
        self.diair['value'] = msg.airspeed
        
    def get_altitude(self, sender, msg):
        self.dialt['var'] = 'altitude'
        self.dialt['shortname'] = 'altitude'
        self.dialt['value'] = msg.alt
        





class PprzSettingsManager:
    def __init__(self, settings_xml_path, ac_id, ivy: IvyMessagesInterface):
        self.ac_id = ac_id
        self.ivy = ivy
        self.settings_grp = PprzSettingsParser.parse(settings_xml_path)
        self.ivy.subscribe(self.update_values, PprzMessage('ground', 'DL_VALUES'))
        #self.ivy.subscribe(self.get_airspeed, PprzMessage('ground', 'FLIGHT_PARAM'))
        

    def update_values(self, sender, msg):
        if self.ac_id == msg.ac_id:
            for i, value in enumerate(msg.values):
                if value != '?':
                    value = float(value)
                    s = self.settings_grp[i] # type: PprzSetting
                    #print(self.settings_grp[i])
                    s.set(value)


    def __getitem__(self, item):
        return self.settings_grp.__getitem__(item)

    def __setitem__(self, key, value):
        setting = self.settings_grp[key]
        msg = PprzMessage("ground", "DL_SETTING")
        msg['ac_id'] = self.ac_id
        msg['index'] = setting.index
        msg['value'] = value
        self.ivy.send(msg)

    def __len__(self):
        return self.settings_grp.__len__()


class PprzSettingsParser:
    def __init__(self):
        self.setting_nb = 0

    @staticmethod
    def parse(settings_xml_path):
        parser = PprzSettingsParser()
        settings_tree = etree.parse(settings_xml_path)
        sets_element = settings_tree.find("dl_settings")
        return parser.parse_dl_settings(sets_element)

    def parse_dl_setting(self, element):
        var = get_attrib(element, "var")
        min_v = get_attrib(element, "min", typ=float)
        max_v = get_attrib(element, "max", typ=float)
        shortname = get_attrib_default(element, "shortname", var)
        step = get_attrib_default(element, "step", 1, typ=float)
        values = get_attrib_default(element, "values", None)

        if values is None:
            values = []
        else:
            values = values.split("|")
            count = int((max_v - min_v + step) / step)
            if len(values) != count:
                print("Warning: possibly wrong number of values ({}) for {} (expected {})"
                      .format(len(values), shortname, count))
        setting = PprzSetting(var, self.setting_nb, shortname, min_v, max_v, step, values, element)

        for e_button in element.findall("strip_button"):
            name = get_attrib(e_button, "name")
            value = get_attrib(e_button, "value")
            icon = get_attrib_default(e_button, "icon", None)
            group = get_attrib_default(e_button, "group", None)

            button = StripButton(value, name, icon, group)
            setting.buttons.append(button)

        for e_key in element.findall("key_press"):
            key = get_attrib(e_key, "key")
            value = get_attrib(e_key, "value")
            key_press = KeyPress(key, value)
            setting.key_press.append(key_press)

        self.setting_nb += 1
        return setting

    def parse_dl_settings(self, element):
        name = get_attrib_default(element, "name", None)
        group = PprzSettingsGrp(name)
        for child in element.getchildren():
            if child.tag == "dl_settings":
                setting_group = self.parse_dl_settings(child)
                group.groups_list.append(setting_group)
            elif child.tag == "dl_setting":
                setting = self.parse_dl_setting(child)
                group.settings_list.append(setting)
            else:
                raise Exception("tag {} not supported!".format(child.tag))
        return group


class PprzSetting:
    """Paparazzi Setting Class"""

    def __init__(self, var, index, shortname, min_value, max_value, step, values, xml):
        self.var = var              # type: str
        self.index = index          # type: int
        self.shortname = shortname  # type: str
        self.min_value = min_value  # type: float
        self.max_value = max_value  # type: float
        self.step = step            # type: float
        self.values = values        # type: List[str]
        self.value = None           # type: float
        self.buttons = []           # type: List[StripButton]
        self.key_press = []         # type: List[KeyPress]
        self.xml = xml

    def __str__(self):
        return "{{var: {}, shortname: {}, index: {}}}".format(self.var, self.shortname, self.index)

    def value_from_name(self, name):
        """Return the index in 'values' table matching a given name. Raise ValueError if the name is unknown."""
        if self.values is None:
            raise ValueError("No named values in this setting")
        return self.values.index(name) + self.min_value

    def set(self, val):
        if isinstance(val, str):
            self.value = self.value_from_name(val)
        elif isinstance(val, float) or isinstance(val, int):
            self.value = val
        else:
            raise Exception("Bad type!")


class StripButton:
    def __init__(self, value, name, icon, group):
        self.value = value
        self.name = name
        self.icon = icon
        self.group = group


class KeyPress:
    def __init__(self, key, value):
        self.value = value
        self.key = key


class PprzSettingsGrp:
    """"Paparazzi Setting Group Class"""

    def __init__(self, name):
        self.name = name
        self.settings_list = []     # type: List[PprzSetting]
        self.groups_list = []       # type: List[PprzSettingsGrp]

    def __str__(self):
        grp_names = [grp.name for grp in self.groups_list]
        settings = [str(setting) for setting in self.settings_list]
        return ", ".join(grp_names + settings)

    def get_all_settings(self):
        all_settings = []
        all_settings += self.settings_list
        for group in self.groups_list:
            all_settings += group.get_all_settings()
        return sorted(all_settings, key=lambda s: s.index)

    def __getitem__(self, item):
        if type(item) == int:
            for setting in self.get_all_settings():
                if item == setting.index:
                    return setting
            raise IndexError("No setting #{}".format(item))
        elif type(item) == str:
            for setting in self.get_all_settings():
                if item == setting.shortname:
                    return setting
            raise AttributeError('No such setting named "{}"'.format(item))

    def __len__(self):
        return len(self.get_all_settings())



def example1():
    # Usage: ./setting.py <path_var_AC>/settings.xml <ac_id>
    
    list_of_totalenergy = []
    list_of_derive_totalenergy = []
    P = []
    abscisse, abscisse2, abscisse3 = [], [], []
    Lx, Ly, Lz, Lz_dot = [], [], [], []
    i = 0
    
    ivy = IvyMessagesInterface("DemoSettings")
    try:
        setting_manager = PprzSettingsManager(sys.argv[1], sys.argv[2], ivy)
        #print(setting_manager)
        setting_manager2 = PprzSettingsManager2(sys.argv[2], ivy)
        setting_manager3 = PprzSettingsManager3(sys.argv[2], ivy)
        setting_manager4 = PprzSettingsManager4(sys.argv[2], ivy)
        
        plt.show()
        while True:
            
            time.sleep(0.8)
            speed = float(setting_manager2.diair['value'])
            altitude = float(setting_manager2.dialt['value'])
            
            #x = float(setting_manager3.dix['value'])
            #y = float(setting_manager3.diy['value'])
            
            #mean_aspeed = float(setting_manager4.diest['value'])
            #Lz_dot.append(mean_aspeed)
            
            
            
            #Lx.append(x)
            #Ly.append(y)
            #Lz.append(altitude)
            
            
                        
            #print(Lx,Ly,Lz)
            
            
            #print(setting_manager["airspeed"])

            energy = altitude + 0.5 * speed * speed / 9.81

            
            list_of_totalenergy.append(energy)
            taille = len(list_of_totalenergy)
            abscisse.append(taille)
            if taille >= 2:
                derive = (list_of_totalenergy[taille - 1] - list_of_totalenergy[taille - 2]) / (abscisse[taille - 1] - abscisse[taille - 2])
                list_of_derive_totalenergy.append(derive)
                abscisse2.append((abscisse[taille - 1] + abscisse[taille - 2]) / 2)
                #print(list_of_derive_totalenergy, abscisse2)
                taille2 = len(list_of_derive_totalenergy)
                
                if taille2 >= 2:
                    i += 1
                    derive2 = (list_of_derive_totalenergy[taille2 - 1] - list_of_derive_totalenergy[taille2 - 2]) / (abscisse2[taille2 - 1] - abscisse2[taille2 - 2])
                    P.append(derive2)
                    abscisse3.append((abscisse[taille - 1] + abscisse[taille - 2]) / 2)
                    
                    
                    #Methode brute
                    #if derive2 <= -0.2:
                    #    bankangle = 22
                    #if derive2 >= 0.2:
                    #    bankangle = 6
                    #if derive2 > -0.2 and derive2 < 0.2:
                    #    bankangle = 10
                        

                    setting_manager["FP_ROLL"] = (16 - 23 * derive2) 


                    #if i == 200:
                            #fig = plt.figure()
                            #ax = fig.add_subplot(111, projection='3d')
                            #ax.scatter(Lx, Ly, Lz)
                            #plt.show()
                            #plt.close()
                            
                            #plt.plot(abscisse, Lz_dot)
                            #plt.show()

            
                    
            
            
            
    except KeyboardInterrupt:
        ivy.shutdown()
        print("Stopping on request")



    
    

            
    except KeyboardInterrupt:
        ivy.shutdown()
        print("Stopping on request")
        
if __name__ == "__main__":
    # example1 usage: ./setting.py <path_var_AC>/settings.xml
    example1()

    # example2 usage: ./setting.py <path_var_AC>/settings.xml <ac_id>
    # example2()

