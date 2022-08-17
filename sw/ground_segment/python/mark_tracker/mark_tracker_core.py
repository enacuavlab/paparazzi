from cmath import sin
from tkinter import Y
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget
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
    MARK_S1: {'id':1, 'name':'DELIVERY'},
    MARK_S2: {'id':2, 'name':'UNK_TAG'},
    MARK_S3: {'id':3, 'name':'AREA_TAG'},  
    MARK_S4: {'id':4, 'name':'SILENT'}
}


class Mark():
    '''
    Store mark information
    '''
    def __init__(self, _id, _name):
        self.id = _id
        self.name = _name
        self.clear()

    def set_pos(self, lat, lon, alt):
        ''' set pos '''
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def clear(self):
        self.lat = None
        self.lon = None
        self.alt = None

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

class Tracker(QWidget,Ui_MarkTracker):
    '''
    Main tracker class
    '''  
    console = pyqtSignal(str)

    def __init__(self, parent=None, verbose=False):
        super().__init__()
        Ui_MarkTracker.__init__(self)
        #self.mark2 = Mark(9, "9")
        self.verbose = verbose
        self.marks_fpl = {}
        self.marks_by_name = {}
        self.known_pos = {
                1: {'lat': 52.171387, 'lon': 4.420618, 'dist': 10},
                2: {'lat': 52.169916, 'lon': 4.415763, 'dist': 100},
                4: {'lat': 52.170707, 'lon': 4.418157, 'dist': 10},
                5: {'lat': 0, 'lon': 0, 'dist': 50},
        }

        self.wps_list = {}
        
        for k, e in mark_types.items():
            self.marks_fpl[k] = Mark(k, e['name'])
            self.marks_by_name[e['name']] = k
        #for i in self.marks_fpl:      
        #    print(i, " ", self.marks_fpl[i])
        
        self.id_list = []
        self.uavs = {}
        #self.uav_id = {}
        self.alt_ref = 0
        self.correct_id = None
        self.connect = None

        self.console.connect(self.console_printer)

    def built(self):
        for i in self.marks_fpl: 
            self.print_console(str(i) + " " + str(self.marks_fpl[i]))
        
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
        self.checkBox_auto_send.setChecked(True)

        ''' get aircraft config '''
        #get current flight plan with its waypoints and UAV info
        def connect_cb(conf):
            global conf2 
            global wps
            conf2 = conf
            active_wps = []

            self.print_console('New A/C {} ({})'.format(conf.name, conf.id))
            try: 
                fpl = FlightPlan().parse(conf.flight_plan)
                wps = fpl.waypoints              
                if self.verbose:
                    for wp in wps:
                        self.print_console('{} {} {}'.format(wp.name, wp.alt, wp.no))
                self.alt_ref = fpl.alt
                self.uavs[conf.name] = fpl
                #if wps is not None:
                #    self.uavs[conf.name] = []
                #    for wp in wps:
                #        self.uavs[conf.name].append(wp.name)
                #        active_wps.append(wp.name)
                #self.uav_id[str(conf.name)] = {'id': conf.id}

            except (IOError, ET.XMLSyntaxError) as e:
                self.print_console('FlightPlan error' + e.__str__())

            if self.verbose:
                print(conf)

            self.active_uav.addItem(conf.name)


        ''' create connect object, it will start Ivy interface '''
        try:
            self.connect = PprzConnect(notify=connect_cb)
        except:
            print("Fail to start PprzConnect")
            sys.exit(0)

        self.search_s3.clicked.connect(self.search_tag_s3)
        self.input_tag_number.returnPressed.connect(self.search_tag_s3)
        #self.input_tag_number.enterEvent.connect(self.search_tag_s3)

        ''' bind to MARK message '''
        def mark_cb(ac_id, msg):
            global conf2, id3
            global wps
            mark_id = int(msg['ac_id']) # abuse ac_id field
            lat = float(msg['lat'])
            lon = float(msg['long'])
            mark = Mark(mark_id, str(ac_id) + "a/c")
            mark.set_pos(lat, lon, self.alt_ref)

            if mark_id not in self.id_list:
                self.id_list.append(mark_id)
                c = self.connect.conf_by_id(str(ac_id))
                self.print_console('New marker {} from {}'.format(mark_id, c.name))

                #find id for the selected waypoint to be moved
                #for i in wps:
                #    if str(self.combo_s3.currentText()) == str(i.name):
                #        no = i.no

                if self.correct_id is not None and int(mark_id) == int(self.correct_id):
                    #MISSION 3
                    #create new marker with the name, id, pos
                    wp = self.uavs[conf2.name].get_waypoint(self.combo_s3.currentText())
                    #mark2_name = wp.name
                    mark.name = "AREA"
                    #self.mark2 = Mark(mark_id,mark2_name)  
                    #self.mark2.set_pos(lat, lon, self.alt_ref)

                    # add correct marker to list
                    if self.combo_s3_wps.findText(str(mark)): #self.mark2)):
                        self.combo_s3_wps.addItem("Found " + str(mark)) #self.mark2))
                    
                    self.print_console("AREA wp moved {}".format(wp.name))

                    id3 = mark.id

                    # update shape position to corresponds its marker
                    #mark_update = Mark(mark_id, "AREA")
                    #mark_update.set_pos(lat, lon, self.alt_ref)
                    self.update_pos_label(mark) #_update)
                    #self.update_shape(mark) #_update)
                    if self.checkBox_auto_send.isChecked():
                        # UPDATE POS LABEL BEFORE SENDING MARKER
                        self.send_mark(MARK_S3)
                
                elif mark.find_distance(self.known_pos[1]['lat'], self.known_pos[1]['lon']) < self.known_pos[1]['dist']:
                    #MISSION 1
                    #mark_update = Mark(no, "DELIVERY")
                    mark.name = "DELIVERY"
                    #mark_update.set_pos(lat, lon, self.alt_ref)
                    self.update_pos_label(mark)
                    self.print_console("DELIVERY found {} {}".format(self.known_pos[1]['lat'], self.known_pos[1]['lon']))
                
                elif mark.find_distance(self.known_pos[4]['lat'], self.known_pos[4]['lon']) < self.known_pos[4]['dist']:
                    #MISSION 4
                    #mark_update = Mark(no, "SILENT")
                    mark.name = "SILENT"
                    #mark_update.set_pos(lat, lon, self.alt_ref)
                    self.update_pos_label(mark)
                    self.print_console("SILENT found {} {}".format(self.known_pos[4]['lat'], self.known_pos[4]['lon']))
                
                elif mark.find_distance(self.known_pos[2]['lat'], self.known_pos[2]['lon']) < self.known_pos[2]['dist']:
                    #MISSION 2
                    #mark_update = Mark(no, "UNKNOWN")
                    wp = self.uavs[conf2.name].get_waypoint(self.combo_s2.currentText())
                    #mark2_name = wp.name
                    mark.name = "UNKNOWN"
                    #self.mark2 = Mark(mark_id, mark2_name)
                    #self.mark2.set_pos(lat, lon, self.alt_ref)
                    #mark_update.set_pos(lat, lon, self.alt_ref)
                    self.update_pos_label(mark)
                    self.print_console("UNK found {} {}".format(self.known_pos[2]['lat'], self.known_pos[2]['lon']))

                    if self.checkBox_auto_send.isChecked():
                        self.send_mark(MARK_S2)
                    
                else:
                    #IF THERE IS A RANDOMLY DETECTED MARKER ON GROUND
                    wp = self.uavs[conf2.name].get_waypoint(self.combo_s3.currentText())
                    #mark2_name = wp.name
                    #self.mark2 = Mark(mark_id,mark2_name)
                    #self.mark2.set_pos(lat, lon, self.alt_ref)

                    #if self.checkBox_auto_send.checkState == True:
                    #    self.send_mark(no)
                    
                    self.print_console("Found unknown marker: " + str(wp.name))

                    #new_mark = Mark(mark_id,"")
                    #new_mark.set_pos(lat,lon,self.alt_ref)

                    if self.combo_s3_wps.findText(str(mark)):
                        self.combo_s3_wps.addItem(str(mark))

                    #mark_update = Mark(no, "")
                    #mark_update.set_pos(lat, lon, self.alt_ref)
                    #self.update_shape(mark_update)
                    #print(self.marks_fpl)              

        self.update_shape(mark)
        self.connect.ivy.subscribe(mark_cb,PprzMessage("telemetry", "MARK"))

    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def print_console(self, words):
        self.console.emit(words)

    def console_printer(self, words):
        self.commands.append(words)
        if self.verbose:
            print(words)

    def search_tag_s3(self):
        self.correct_id = self.input_tag_number.text().__str__()
        self.print_console("Selected tag ID: {}".format(self.correct_id))
        return self.correct_id
    
    def uav_selected(self, i):
        ''' update WP list when changing the selected UAV '''
        if self.verbose:
            self.print_console('A/C selected: ' + str(i))
        wps = [x.name for x in self.uavs[self.active_uav.currentText()].waypoints]

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
            self.print_console("Mission 1 waypoint not found")

        try:
            self.combo_s2.setCurrentIndex(wps.index(self.marks_fpl[MARK_S2].name))
        except:
            self.print_console("Mission 2 waypoint not found")

        try:
            self.combo_s3.setCurrentIndex(wps.index(self.marks_fpl[MARK_S3].name))
        except:
            self.print_console("Mission 3 waypoint not found")
        
        try:
            self.combo_s4.setCurrentIndex(wps.index(self.marks_fpl[MARK_S4].name))
        except:
            self.print_console("Mission 4 waypoint not found")

    def update_pos_label(self, mark):
        self.print_console("Update mark: "+str(mark.id))
        if mark.id is not None:
            if mark.name == "AREA" or mark.name == "":
                self.pos_s3.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif mark.name == "DELIVERY":
                self.id_s1.setText(str(mark.id))
                self.pos_s1.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif mark.name == "UNKNOWN":
                self.id_s2.setText(str(mark.id))
                self.pos_s2.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif mark.name == "SILENT":
                self.id_s4.setText(str(mark.id))
                self.pos_s4.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))

    def clear_pos_label(self, mark):
        self.print_console("Clear mark " + str(mark.id) + " " + str(mark.name))
        def safe_remove(label):
            try:
                _id = int(label.text())
                self.id_list.remove(_id)
            except:
                pass
            finally:
                label.setText(" - ")

        if mark.id is not None:
            if mark.id == MARK_S1:
                self.pos_s1.setText("lat / lon")
                safe_remove(self.id_s1)
            if mark.id == MARK_S2:
                self.pos_s2.setText("lat / lon")
                safe_remove(self.id_s2)
            if mark.id == MARK_S3:
                self.pos_s3.setText("lat / lon") 
                self.id_list.remove(id3)     
            if mark.id == MARK_S4:
                self.pos_s4.setText("lat / lon")  
                safe_remove(self.id_s4)

    def get_wp_id(self, mark_id):
        ''' get WP id from mark id '''
        for i, e in mark_types.items():
            if(mark_id is not None):
                if int(e['id']) == int(mark_id):
                    if i == MARK_S1:
                        self.print_console("MARK_S1 = " + str(mark_id))
                        return self.combo_s1.currentIndex() + 1
                    elif i == MARK_S2:
                        self.print_console("MARK_S2 = " + str(mark_id))
                        return self.combo_s2.currentIndex() + 1
                    elif i == MARK_S3:
                        self.print_console("MARK_S3 = " + str(mark_id))
                        return self.combo_s3.currentIndex() + 1                        
                    elif i == MARK_S4:
                        self.print_console("MARK_S4 = " + str(mark_id))
                        return self.combo_s4.currentIndex() + 1
            else:
                self.print_console("Mark id error!")
                return None

    def send_mark(self, mark_id):
        global wps
        ''' send mark to selected uab cb '''

        #CHANGE MARK TYPE - LOC
        mark = self.marks_fpl[mark_id]
        pos = []
        if mark_id == MARK_S3:
            pos = self.pos_s3.text().split("/")
        elif mark_id == MARK_S2:
            pos = self.pos_s2.text().split("/")
        elif mark_id == MARK_S1:
            pos = self.pos_s1.text().split("/")
        elif mark_id == MARK_S4:
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
                self.move_wp(uav_id, wp_id, mark)
                if self.verbose:
                    self.print_console('Send mark {} to UAV {} ({}), for WP {}'.format(mark.name, uav_name, uav_id, wp_id))

                self.print_console("Mark id {} vs Correct id {}".format(mark_id, self.correct_id))

            except Exception as e:
                if self.verbose:
                    self.print_console('Send_mark error:' + str(e))

    def clear_mark(self, mark_id):
        ''' clear mark cb '''
        # print(self.marks_fpl)
        mark = self.marks_fpl[mark_id]
        mark.clear()
        self.clear_shape(mark)
        if self.verbose:
            self.print_console('Clear marker - ' + mark.name)

    def move_wp(self, ac_id, wp_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        msg['alt'] = mark.alt
        self.print_console("MOVING WP {} for A/C {}, Mark {} ({}, {})".format(wp_id, ac_id, mark.name, mark.lat, mark.lon))
        self.connect.ivy.send(msg)

    def update_shape(self, mark):
        ''' create or update a shape on the GCS map '''
        #if(mark.name != ""):
        #    self.update_pos_label(id, mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['linecolor'] = 'red'
        msg['opacity'] = 0 # fill color
        msg['shape'] = 0 # circle
        msg['status'] = 0 # create or update
        msg['latarr'] = [int(10**7 * mark.lat),0]
        msg['lonarr'] = [int(10**7 * mark.lon),0]
        msg['radius'] = 2.
        msg['text'] = 'Tag_ID_{}'.format(mark.id)
        self.print_console("UPDATE SHAPE {} ({}, {})".format(mark.id, mark.lat, mark.lon))
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

