from cmath import sin
from tkinter import Y
from PyQt5 import QtCore, QtWidgets, QtGui
from collections import namedtuple
from ui.mainwindow import Ui_MarkTracker
import traceback
from time import sleep
import numpy as np
from math import atan2, cos, sqrt, pi

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))

sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from flight_plan import FlightPlan
from flight_plan import Waypoint



import lxml.etree as ET

#only two types of markers, one vector type with all the markers and one variable holding the correct tag value
#dynamic vector which appends new positions of ArUco tags

MARK_S1 = 1
MARK_S2 = 2
MARK_S3 = 3
MARK_S4 = 4
mark_types = {
    MARK_S1: {'id':1, 'name':'Known location'},
    MARK_S2: {'id':2, 'name':'100m GPS Spoof'},
    MARK_S3: {'id':3, 'name':'Unknown field location'},  
    MARK_S4: {'id':4, 'name':'Silent Deilivery'}
}


class Mark():
    '''
    Store mark information
    '''
    def __init__(self, _id, _name):
        self.id = _id
        self.name = _name
        self.lat = 0.
        self.lon = 0.
        self.alt = 0.
        self.lat_arr = np.array([])
        self.lon_arr = np.array([])

    def set_pos(self, lat, lon, alt):
        ''' set pos '''
        # filtering
        self.lat_arr = np.append(self.lat_arr, [lat])
        self.lon_arr = np.append(self.lon_arr, [lon])
        self.lat = np.mean(self.lat_arr)
        self.lon = np.mean(self.lon_arr)
        self.alt = alt

    def clear(self):
        self.lat_arr = np.array([])
        self.lon_arr = np.array([])

    def __str__(self):
        out = 'Mark {} with ID {} at pos {}, {}'.format(self.name, self.id, self.lat, self.lon)
        return out
    
    def find_distance(self, lat, lon):
        # R = 6371 * 1000
        # phi1 = lat * pi / 180
        # phi2 = self.lat * pi / 180
        # delta_phi = (self.lat - lat) * pi / 180
        # delta_l = (self.lon - lon) * pi / 180
        # a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi1) * cos(phi2) * sin(delta_l/2) * sin(delta_l/2)
        # c = 2 * atan2(sqrt(a),sqrt(1-a))
        # dist = d = R * c

        x = (lon - self.lon) * cos(pi * (lat + self.lat) / (2. * 180.))
        y = lat - self.lat
        z = sqrt(x*x + y*y) # "distance" in degree
        dist = 1852 * 60 * z

        return dist

class Tracker(Ui_MarkTracker):
    '''
    Main tracker class
    '''  
    def __init__(self, parent=None, verbose=False):
        Ui_MarkTracker.__init__(self)
        self.mark2 = Mark(9, "9")
        self.verbose = verbose
        self.marks_fpl = {}
        self.marks_by_name = {}
        self.known_pos = {
            1: {'lat':52.171387, 'lon': 4.420618},
            2: {'lat': 52.169916, 'lon': 4.415763},
            4: {'lat': 52.170707, 'lon': 4.418157},
            5: {'lat': 0, 'lon': 0},
        }
        
        for k, e in mark_types.items():
            self.marks_fpl[k] = Mark(k, e['name'])
            self.marks_by_name[e['name']] = k
        for i in self.marks_fpl:      
            print(i, " ", self.marks_fpl[i])
        
        self.id_list = []
        #print(mark_types)
        #print(self.marks)
        self.uavs = {}
        self.uav_id = {}
        self.alt_ref = 0

    def built(self):
        for i in self.marks_fpl: 
            hist = str(i) + " " + str(self.marks_fpl[i])
            self.commands.append(hist)
        
        
        ''' HMI callbacks '''
        self.active_uav.currentIndexChanged.connect(self.uav_selected)
        self.clear_s1.clicked.connect(lambda:self.clear_mark(MARK_S1))
        self.clear_s2.clicked.connect(lambda:self.clear_mark(MARK_S2))
        self.clear_s3.clicked.connect(lambda:self.clear_mark(MARK_S3))
        self.clear_s4.clicked.connect(lambda:self.clear_mark(MARK_S4))
        self.send_s1.clicked.connect(lambda:self.send_mark(MARK_S1))
        self.send_s2.clicked.connect(lambda:self.send_mark(MARK_S2))
        self.send_s3.clicked.connect(lambda:self.send_mark(MARK_S3))
        self.send_s4.clicked.connect(lambda:self.send_mark(MARK_S4))
        
        ''' get aircraft config '''
        #get current flight plan with its waypoints and UAV info
        def connect_cb(conf):
            global conf2 
            global wps
            conf2 = conf
            active_wps = []

            try: 
                fpl = FlightPlan().parse(conf.flight_plan)
                wps = fpl.waypoints              
                for wp in wps:
                    print(wp.name, " ", wp.alt, " ", wp.no)
                hist = str(wps) + "\n"
                self.commands.append(hist)
                self.alt_ref = fpl.alt
                if wps is not None:
                    self.uavs[conf.name] = []
                    for wp in wps:
                        self.uavs[conf.name].append(wp.name)
                        active_wps.append(wp.name)
                self.uav_id[str(conf.name)] = {'id': conf.id}

            except (IOError, ET.XMLSyntaxError) as e:
                hist = 'FlightPlan error' + e.__str__()
                self.commands.append(hist)
                print('FlightPlan error',e.__str__())

            if self.verbose:
                print(conf)

            self.active_uav.addItem(conf.name)

            hist = str(self.uavs[conf.name])
            self.commands.append(hist)
            print(self.uavs[conf.name]," ################################### ")

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        self.search_s3.clicked.connect(self.search_tag_s3)
        self.input_tag_number.returnPressed.connect(self.search_tag_s3)
        #self.input_tag_number.enterEvent.connect(self.search_tag_s3)

        ''' bind to MARK message '''
        def mark_cb(ac_id, msg):
            global conf2, id3
            global correct_id
            global wps
            mark_id = int(msg['ac_id']) # abuse ac_id field
            marker = "ACTIVE"
            if(mark_id is not None):
                if(mark_id not in self.id_list):
                    self.id_list.append(mark_id)
                    hist = "NEW MARKER DETECTED!"
                    self.commands.append(hist)

                    if self.verbose:
                        hist = 'From ' + str(ac_id) + ': ' + str(msg)
                        self.commands.append(hist)
                        print('from ',ac_id,':',msg)

                    lat = float(msg['lat'])
                    lon = float(msg['long'])
                    
                    mark = Mark(mark_id, str(ac_id) + "a/c")
                    mark.set_pos(lat, lon, self.alt_ref)

                    #find id for the selected UAV
                    for i, j in self.uav_id.items():
                        if(i == self.active_uav.currentText()):
                            print("UAV ID is: ", j['id'])
                            id = j['id']

                    #find id for the selected waypoint to be moved
                    print("WAYPOINTS")
                    for i in wps:
                        print(i.name, " ", i.no)
                        if(str(self.combo_s3.currentText()) == str(i.name)):
                            no = i.no
                            print("CORRECT ", i.name, ": ", i.no)
                    if(int(mark_id) == int(correct_id)):
                        #MISSION 3                        
                        print("DICT HERE: ", self.uav_id)                      
                        #create new marker with the name, id, pos
                        mark2_name = self.uavs[conf2.name][no-1]
                        self.mark2 = Mark(mark_id,mark2_name)  
                        self.mark2.set_pos(lat, lon, self.alt_ref)
                        print(mark2_name, " ", lat, " ", lon)

                        # add correct marker to list
                        if self.combo_s3_wps.findText(str(self.mark2)):
                            self.combo_s3_wps.addItem("Found " + str(self.mark2))
                        
                        hist = "This is the correct marker moved as: " + str(mark2_name)
                        self.commands.append(hist)

                        self.connect.ivy.send(str(self.marks_fpl[MARK_S3]))

                        
                        id3 = mark.id

                        # update shape position to corresponds its marker
                        mark_update = Mark(no, "S3")
                        mark_update.set_pos(lat, lon, self.alt_ref)
                        self.update_shape(mark_id, mark_update)
                        if self.checkBox_auto_send.isChecked():
                            print("SENDING MARKER NOW S3")
                            # UPDATE POS LABEL BEFORE SENDING MARKER
                            self.send_mark(MARK_S3)
                            # move the selected waypoint on the GCS
                            # self.move_wp(id, no, self.mark2)
                         
                    
                    elif mark.find_distance(self.known_pos[1]['lat'], self.known_pos[1]['lon']) < 5:
                        #MISSION 1
                        mark_update = Mark(no, "S1")
                        mark_update.set_pos(lat, lon, self.alt_ref)
                        self.update_pos_label(mark_id, mark_update)
                        print("S1 found ", self.known_pos[1]['lat']," ", self.known_pos[1]['lon'])
                    
                    elif mark.find_distance(self.known_pos[4]['lat'], self.known_pos[4]['lon']) < 5:
                        #MISSION 4
                        mark_update = Mark(no, "S4")
                        mark_update.set_pos(lat, lon, self.alt_ref)
                        self.update_pos_label(mark_id, mark_update)
                        print("S4 found ", self.known_pos[4]['lat']," ", self.known_pos[1]['lon'])
                    
                    elif mark.find_distance(self.known_pos[2]['lat'], self.known_pos[2]['lon']) < 100:
                        #MISSION 2
                        mark_update = Mark(no, "S2")
                        mark2_name = self.uavs[conf2.name][no]
                        self.mark2 = Mark(mark_id, mark2_name)
                        self.mark2.set_pos(lat, lon, self.alt_ref)
                        mark_update.set_pos(lat, lon, self.alt_ref)
                        self.update_shape(mark_id, mark_update)
                        print(mark2_name, " ", lat, " ", lon)

                        if self.checkBox_auto_send.isChecked():
                            print("SENDING MARKER NOW S2")
                            self.send_mark(MARK_S2)
                            #self.move_wp(id, no, self.mark2)
                        
                        print("S2 found ", self.known_pos[2]['lat']," ",self.known_pos[1]['lon'])
                    
                    else:
                        #IF THERE IS A RANDOMLY DETECTED MARKER ON GROUND
                        print(no)
                        mark2_name = self.uavs[conf2.name][no]
                        self.mark2 = Mark(mark_id,mark2_name)
                        self.mark2.set_pos(lat, lon, self.alt_ref)
                        print(mark2_name, " ", lat, " ", lon)

                        if self.checkBox_auto_send.checkState == True:
                            self.send_mark(no)
                            # self.move_wp(id, no, self.mark2)  
                        
                        hist = "This is a wrong marker moved as: " + str(mark2_name)
                        self.commands.append(hist)

                        new_mark = Mark(mark_id,"")
                        new_mark.set_pos(lat,lon,self.alt_ref)

                        if self.combo_s3_wps.findText(str(new_mark)):
                            self.combo_s3_wps.addItem(str(new_mark))

                        mark_update = Mark(no, "")
                        mark_update.set_pos(lat, lon, self.alt_ref)
                        self.update_shape(mark_id, mark_update)
                        print(self.marks_fpl)              
            else:
                hist = "Marker found is not a marker type"
                self.commands.append(hist)
                print("Marker found is not a marker type")
        print("IVY")
        self.connect.ivy.subscribe(mark_cb,PprzMessage("telemetry", "MARK"))

        # self.connect.ivy.subscribe(self.search_tag_s3)

        def telemetry(ac_id, msg):
            mark_id = int(msg['ac_id'])
            wp_id = int(msg['wp_id'])
            lat = float(msg['lat'])
            lon = float(msg['long'])
            alt = float(msg['alt'])
            print(mark_id, " ", wp_id, " ", lat, " ", lon, " ", alt)
        self.connect.ivy.subscribe(telemetry,PprzMessage("ground", "MOVE_WAYPOINT"))
    
    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def search_tag_s3(self):
        global correct_id
        correct_id = self.input_tag_number.text().__str__()
        hist = "Input id: " + correct_id  + "\n"
        self.commands.append(hist)
        print(correct_id)
        return correct_id 
    
    def uav_selected(self, i):
        ''' update WP list when changing the selected UAV '''
        if self.verbose:
            hist = 'Selected ' + str(i)
            self.commands.append(hist)
            print('selected',i)
        wps = self.uavs[self.active_uav.currentText()]

        self.combo_s1.clear()
        self.combo_s1.addItems(wps)
        self.combo_s2.clear()
        self.combo_s2.addItems(wps)
        self.combo_s3.clear()
        self.combo_s3.addItems(wps)
        self.combo_s4.clear()
        self.combo_s4.addItems(wps)
        #print(self.marks_fpl[MARK_S1].name)
        
        try:
            self.combo_s1.setCurrentIndex(wps.index(self.marks_fpl[MARK_S1].name))
        except:
            if self.verbose:
                hist = "Mission 1 waypoint not found"
                self.commands.append(hist)
                print("Mission 1 waypoint not found")

        try:
            self.combo_s2.setCurrentIndex(wps.index(self.marks_fpl[MARK_S2].name))
        except:
            if self.verbose:
                hist = "Mission 2 waypoint not found"
                self.commands.append(hist)
                print("Mission 2 waypoint not found")

        try:
            self.combo_s3.setCurrentIndex(wps.index(self.marks_fpl[MARK_S3].name))
        except:
            if self.verbose:
                hist = "Mission 3 waypoint not found"
                self.commands.append(hist)
                print("Mission 3 waypoint not found")
        
        try:
            self.combo_s4.setCurrentIndex(wps.index(self.marks_fpl[MARK_S4].name))
        except:
            if self.verbose:
                hist = "Mission 4 waypoint not found"
                self.commands.append(hist)
                print("Mission 4 waypoint not found")

    def update_pos_label(self, id, mark):
        hist = "Label update" + "\n"
        self.commands.append(hist)
        if(mark.id is not None):
            if(mark.name == "S3" or mark.name == ""):
                self.pos_s3.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S1"):
                self.id_s1.setText(str(id))
                self.pos_s1.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S2"):
                self.id_s2.setText(str(id))
                self.pos_s2.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S4"):
                self.id_s4.setText(str(id))
                self.pos_s4.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
        else:
            hist = "Mark id error"
            self.commands.append(hist)

    def clear_pos_label(self, mark):
        print(mark.id, " ", mark.name, " ", MARK_S1, " ", MARK_S3)
        if(mark.id is not None):
            if(mark.id == MARK_S1):
                self.pos_s1.setText("lat / lon")
                id = int(self.id_s1.text())
                self.id_list.remove(id) 
                self.id_s1.setText(" - ")            
            if(mark.id == MARK_S2):
                self.pos_s2.setText("lat / lon")  
                id = int(self.id_s2.text())
                self.id_list.remove(id) 
                self.id_s2.setText(" - ")            
            if(mark.id == MARK_S3):
                self.pos_s3.setText("lat / lon") 
                self.id_list.remove(id3)     
            if(mark.id == MARK_S4):
                self.pos_s4.setText("lat / lon")  
                id = int(self.id_s4.text())
                self.id_list.remove(id) 
                self.id_s4.setText(" - ")                 
        else:
            hist = "Mark id error"
            self.commands.append(hist)

    def get_wp_id(self, mark_id):
        ''' get WP id from mark id '''
        for i, e in mark_types.items():
            if(mark_id is not None):
                if(int(e['id']) == int(mark_id)):
                    if( i == MARK_S1):
                        hist = "MARK_S1 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S1", " = ", mark_id)
                        return self.combo_s1.currentIndex() + 1
                    elif (i == MARK_S2):
                        hist = "MARK_S2 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S2", " = ", mark_id)
                        return self.combo_s2.currentIndex() + 1
                    elif (i == MARK_S3):
                        hist = "MARK_S3 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S3", " = ", mark_id)
                        return self.combo_s3.currentIndex() + 1                        
                    elif (i == MARK_S4):
                        hist = "MARK_S4 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S4", " = ", mark_id)
                        return self.combo_s4.currentIndex() + 1
            else:
                hist = "Mark id error!"
                self.commands.append(hist)
                return None

    def send_mark(self, mark_id):
        global correct_id, wps
        ''' send mark to selected uab cb '''
        hist = "Send mark - " + str(mark_id)
        self.commands.append(hist)
        print("Send_mark - ",mark_id)


        #CHANGE MARK TYPE - LOC
        mark = self.marks_fpl[mark_id]
        pos = []
        if (mark_id == MARK_S3):
            pos = self.pos_s3.text().split("/")
        elif (mark_id == MARK_S2):
            pos = self.pos_s2.text().split("/")
        elif (mark_id == MARK_S1):
            pos = self.pos_s1.text().split("/")
        elif (mark_id == MARK_S4):
            pos = self.pos_s4.text().split("/")
            
        if pos is not None:
            lat = float(pos[0])
            lon = float(pos[1])
            mark.set_pos(lat,lon,self.alt_ref)

        uav_name = self.active_uav.currentText()
        wp_id = self.get_wp_id(mark_id)
        
        if uav_name != '':
            try:
                uav_id = self.connect.conf_by_name(uav_name).id
                print("WP WILL BE MOVED")
                self.move_wp(uav_id, wp_id, mark)
                if self.verbose:
                    hist = 'Send mark {} to UAV {} ({}), for WP {}'.format(mark.name, uav_name, uav_id, wp_id)
                    self.commands.append(hist)
                    print('Send mark {} to UAV {} ({}), for WP {}'.format(mark.name, uav_name, uav_id, wp_id))
                
                hist = "Mark id " + mark_id.__str__() + " vs Correct id " + correct_id.__str__() + "\n"
                self.commands.append(hist)

            except Exception as e:
                if self.verbose:
                    hist = 'Send_mark error:' + e.__str__()  + "\n"
                    self.commands.append(hist)
                    print('Send_mark error:', e.__str__())

    def clear_mark(self, mark_id):
        ''' clear mark cb '''
        # print(self.marks_fpl)
        mark = self.marks_fpl[mark_id]
        mark.clear()
        self.clear_shape(mark)
        if self.verbose:
            hist = 'Clear marker - ' + mark.name + "\n"
            self.commands.append(hist)
            print('Clear marker - ', mark.name,"\n");

    def move_wp(self, ac_id, wp_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        msg['alt'] = mark.alt
        print("WPOINT MOVED ")
        print("MOVING WP ", wp_id, " AC ID ", ac_id, " NAME ", mark.name, " LAT ", mark.lat, " LON ", mark.lon)
        self.connect.ivy.send(msg)

    def update_shape(self, id, mark):
        ''' create or update a shape on the GCS map '''
        if(mark.name != ""):
            self.update_pos_label(id, mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['opacity'] = 1 # fill color
        msg['shape'] = 0 # circle
        msg['status'] = 0 # create or update
        msg['latarr'] = [int(10**7 * mark.lat),0]
        msg['lonarr'] = [int(10**7 * mark.lon),0]
        msg['radius'] = 2.
        msg['text'] = mark.name
        print("UPDATE SHAPE ", mark.id, " ", mark.lat, " ", mark.lon)
        self.connect.ivy.send(msg)

    def clear_shape(self, mark):
        ''' delete a shape on the GCS map '''
        self.clear_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['opacity'] = 0 # no fill color
        msg['shape'] = 0 # circle
        msg['status'] = 1 # delete
        msg['latarr'] = [0]
        msg['lonarr'] = [0]
        msg['radius'] = 0.
        msg['text'] = 'NULL'
        self.connect.ivy.send(msg)

