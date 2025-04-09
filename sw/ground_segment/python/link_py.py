#!/usr/bin/env python3

"""
/home/pprz/Projects/paparazzi/sw/ground_segment/tmtc/link_py.py  
   -ac 120:127.0.0.1:4242:4243 
   -d /dev/paparazzi/xbee -s 57600 -t xbee
   -ac 120:127.0.0.1:4242:4243 -d /dev/paparazzi/xbee -s 57600 -t xbee 

This program aims to be used in place of link.ml, link_combiner.py, pprzlink_proxy.py
"""

import os
import sys

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../../../../')))
sys.path.append(PAPARAZZI_HOME + "/var/lib/python")

from ivy.std_api import *
import threading
import time

import pprzlink.serial
import pprzlink.udp
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message

DEFAULT_BROADCAST= "127.255.255.255"
REPEAT="Repeat"
IP_BCAST="IPBcast"

PING_PERIOD = 5.0
STATUS_PERIOD = 1.0

BUFFER_SIZE = 10 #The number of elements messages to be stored in the circular buffer for each link.


class DownLinkStatus():

  def __init__(self, ac_id, address='127.0.0.1'):
    self.ac_id = ac_id
    self.address = address
    self.rx_bytes = 0
    self.rx_msgs = 0
    self.tx_msgs = 0
    self.run_time = 0
    self.last_rx_bytes = 0
    self.last_rx_msgs = 0
    self.last_msg_time = 0
    self.last_ping_time = 0
    self.last_pong_time = 0


#class UDPLink_REF:
#    def __init__(self,opts):
#        messages_xml_map.parse_messages()
#        self.run_event = threading.Event()
#        self.uplink_port = opts.uplink_port
#        self.downlink_port = opts.downlink_port
#        self.udp = pprzlink.udp.UdpMessagesInterface(self.proccess_downlink_msg, False, self.uplink_port, self.downlink_port)
#        self.ivy = pprzlink.ivy.IvyMessagesInterface("LinkPy", True, False, opts.bus)
#        self.ac_downlink_status = {}
#        self.rx_err = 0
#        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
#        self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
#        self.bcast_method = opts.broadcast_method
#        self.bcast_addr = opts.broadcast_address
#
#    def updateStatus(self, ac_id, length, address, isPong):
#        if ac_id not in self.ac_downlink_status:
#            self.ac_downlink_status[ac_id] = DownLinkStatus(ac_id, address)
#        self.ac_downlink_status[ac_id].rx_msgs += 1
#        self.ac_downlink_status[ac_id].rx_bytes += length
#        self.ac_downlink_status[ac_id].last_msg_time = time.time()
#        if isPong:
#            self.ac_downlink_status[ac_id].last_pong_time = time.time() - self.ac_downlink_status[ac_id].last_ping_time
#
#    def proccess_downlink_msg(self,sender,address,msg,length,receiver_id=None, component_id=None):
#        if self.run_event.is_set():
#            # print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
#            self.ivy.send(msg,sender,receiver_id,component_id)
#            self.updateStatus(sender, length, address,msg.name == "PONG")
#
#    def proccess_uplink_msg(self,ac_id,msg):
#        # print ('New IVY message to %s : %s' % (ac_id,msg))
#        if msg.broadcasted:
#            if self.bcast_method==IP_BCAST:
#                self.udp.send(msg,0,self.bcast_addr)
#                #XP self.udp.send(msg,0,(self.bcast_addr,self.uplink_port))
#            else:
#                for dest in self.ac_downlink_status.keys():
#                    self.udp.send(msg, 0, (self.ac_downlink_status[dest].address[0], self.uplink_port))
#                    self.ac_downlink_status[dest].tx_msgs += 1
#        else:
#            if isinstance(ac_id,str):
#                ac_id = int(ac_id)
#            # Only send message if the ac is known
#            if ac_id in self.ac_downlink_status:
#                self.udp.send(msg,0,self.bcast_addr)
#                #XP self.udp.send(msg,0,(self.ac_downlink_status[ac_id].address[0],self.uplink_port),ac_id)
#                self.ac_downlink_status[ac_id].tx_msgs+=1
#            else:
#                print ('Message for unknown ac %d' % ac_id)
#
#    def initial_ivy_binds(self):
#        # Subscribe to all datalink messages
#        messages_datalink = messages_xml_map.get_msgs("datalink")
#        for msg in messages_datalink:
#            self.ivy.subscribe(self.proccess_uplink_msg, message.PprzMessage("datalink", msg))
#
#    def run(self):
#        print ('Starting UDPLink for protocol version %s' % (messages_xml_map.PROTOCOL_VERSION))
#        self.udp.start()
#        self.ivy.start()
#
#        self.run_event.set()
#
#        self.status_timer.start()
#        self.ping_timer.start()
#
#        self.initial_ivy_binds()
#
#        try:
#            while True:
#                time.sleep(.5)
#        except KeyboardInterrupt:
#            print ("Stopping UDPLink.")
#            self.status_timer.cancel()
#            self.ping_timer.cancel()
#            self.run_event.clear()
#            # t.join()
#            self.udp.stop()
#            self.ivy.stop()
#            self.udp.join()
#
#    def sendPing(self):
#        for (ac_id, value) in self.ac_downlink_status.items():
#            if messages_xml_map.PROTOCOL_VERSION=="2.0":
#                # For pprzlink V2.0 set the receiver id
#                self.udp.send(message.PprzMessage('datalink','PING'), 0, DEFAULT_BROADCAST)
#                #XP self.udp.send(message.PprzMessage('datalink','PING'), 0, self.ac_downlink_status[int(ac_id)].address, ac_id)
#            else:
#                self.udp.send(message.PprzMessage('datalink','PING'),0,self.ac_downlink_status[int(ac_id)].address)
#            value.last_ping_time = time.time()
#        self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
#        self.ping_timer.start()
#
#    def sendStatus(self):
#        for (key, value) in self.ac_downlink_status.items():
#            self.ivy.send("link LINK_REPORT %i %i %i %i %i %i %i %i %i %i %i" % (
#                value.ac_id,
#                -1,
#                value.run_time,
#                time.time() - value.last_msg_time,
#                value.rx_bytes,
#                value.rx_msgs,
#                self.rx_err,
#                value.rx_bytes - value.last_rx_bytes,
#                value.rx_msgs - value.last_rx_msgs,
#                value.tx_msgs,
#                1000 * value.last_pong_time))
#            value.last_rx_bytes = value.rx_bytes
#            value.last_rx_msgs = value.rx_msgs
#            value.run_time = value.run_time + 1
#
#        self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
#        self.status_timer.start()
#

class Circular_Buffer:

  def __init__(self):
    self.index = 0
    self.size = BUFFER_SIZE
    self.buffer = [""]*self.size

  def add(self, contents):
    self.buffer[self.index] = contents
    if self.index >= (self.size - 1):
      self.index = 0
    else:
      self.index += 1


  def contains(self, contents):
    counter = self.index
    while 1:
      if self.buffer[counter] == contents:
        return 1
      else:
        if counter <= 0:
          counter = self.size-1    
          # Moving the counter in the reverse direction of the index in order to test 
          # the most recent contents first (since they're most likely to match)
        else:
          counter -= 1
        if counter == self.index:
          return 0


  def remove(self, contents):
    counter = self.index
    while 1:
      if self.buffer[counter] == contents:
        self.buffer[counter] = ""
      else:
        if counter <= 0:
          counter = self.size-1
        else:
          counter -= 1
        if counter == self.index:
          return

  def displayContents(self):
    for counter in range(0,self.size):
      if self.index != counter:
        print("   [%s]" %self.buffer[counter])
      else:
        print("-> [%s]" %self.buffer[counter])



class Message:

  def __init__(self, link_name, sender, msg_name, stringvalues):
    self.link_name = link_name
    self.raw_sender = sender
    self.msgName = msg_name
    self.stringvalues = stringvalues

  def linkName(self):
    return self.link_name

  def message(self):
    tmp = str(self.raw_sender)+" "+self.msgName+" "+self.stringvalues
    return tmp

  def sender(self):
    return self.raw_sender

  def name(self):
    return self.name



class Link:

  def __init__(self, name, ac_id):
    self.buffer = Circular_Buffer()
    self.name = name

  def checkBuffer(self,message):
    return self.buffer.contains(message.message())

  def addToBuffer(self,message):
    self.buffer.add(message.message())
    #self.buffer.displayContents();

  def removeFromBuffer(self,message):
    self.buffer.remove(message.message())



class Link_Combiner:

  def __init__(self,ivy):
    self.ivy=ivy
    self.links = {}

  def send(self,message):
    if message.linkName() not in self.links:
      self.links[message.linkName()] = Link(message.linkName(), message.sender())
    if not self.checkBuffers(message):
      IvySendMsg(message.message())
    self.links[message.linkName()].addToBuffer(message)

  def checkBuffers(self, message):
    # The returned value is the best guess at whether the message is a duplicate (True), or not (False).
    # If the message is already in this link's buffer, then taking it as not a duplicate. So returning False. 
    # But also, removing it from all buffers. So that when they receive it, they don't do the same.
    # If the message is not in this link's buffer, then checking all other buffers and only if it's not in any of them, 
    # counting the message as not a duplicate.
    match = self.links[message.linkName()].checkBuffer(message)
    if match:   #Removing the message from all buffers
      for link_name in self.links:
        self.links[link_name].removeFromBuffer(message)
      return False
    else:       #Checking all other links' buffers
      for link_name in self.links:
        if link_name == message.linkName():
          continue
        else:
           match = self.links[link_name].checkBuffer(message)
        if match:
          return True
        return False

        if match_count == 0:
            return False
        elif match_count == length(self.links):
          for link_name in self.links:
            self.links[link_name].removeFromBuffer(message)



class UDPLink:

  def __init__(self,link_comb,acs):
    self.link_id=1
    self.link_name="UDPLink"
    self.link_comb=link_comb
    self.ac_downlink_status = {}
    self.interfaces = []
    for (ac) in acs:
      self.ac_downlink_status[int(ac[0])] = DownLinkStatus(int(ac[0]))
      self.interfaces.append((int(ac[0]),ac[1],
        pprzlink.udp.UdpMessagesInterface(self.proccess_downlink_msg, False, int(ac[3]), int(ac[2]))))

  def proccess_downlink_msg(self,sender,address,msg,length,receiver_id=None, component_id=None):
    if sender not in self.ac_downlink_status:
      self.ac_downlink_status[sender] = DownLinkStatus(sender)
    msgString=msg.payload_to_ivy_string()
    self.link_comb.send(Message(self.link_name, sender, msg.name, msgString))
    self.ac_downlink_status[sender].rx_msgs += 1
    self.ac_downlink_status[sender].rx_bytes += len(msgString)
    self.ac_downlink_status[sender].last_msg_time = time.time()
    if(msg.name=="PONG"):
      self.ac_downlink_status[sender].last_pong_time = time.time() - self.ac_downlink_status[sender].last_ping_time

  def start(self):
    for elt in self.interfaces: elt[2].start()

  def join(self,arg): 
    for elt in self.interfaces: elt[2].join(arg)

  def stop(self): 
    for elt in self.interfaces: elt[2].stop()

  def send(self,ac_id,msg):
    for elt in self.interfaces: 
      if elt[0] == ac_id:
        elt[2].send(msg,0,elt[1],elt[0]) 
        self.ac_downlink_status[elt[0]].tx_msgs += 1

  def sendPing(self,ping_time):
    msg=message.PprzMessage('datalink','PING')
    for elt in self.interfaces: 
      elt[2].send(msg,0,elt[1],elt[0])
      self.ac_downlink_status[elt[0]].last_ping_time = ping_time



class XBEELink:

  def __init__(self,link_comb,args):
    self.link_id=2
    self.link_name="XBEELink"
    self.alive=False
    self.link_comb=link_comb
    self.ac_downlink_status = {}
    self.interface = pprzlink.serial.XbeeMessagesInterface(lambda s, m: self.proccess_downlink_msg(s, m), 
                                                           0, args.dev, args.baud, verbose=True)

  def proccess_downlink_msg(self,sender,msg):
    if(self.alive):
      if sender not in self.ac_downlink_status: 
        self.ac_downlink_status[sender] = DownLinkStatus(sender)
      msgString=msg.payload_to_ivy_string()
      self.link_comb.send(Message(self.link_name, sender, msg.name, msgString))
      self.ac_downlink_status[sender].rx_msgs += 1
      self.ac_downlink_status[sender].rx_bytes += len(msgString)
      self.ac_downlink_status[sender].last_msg_time = time.time()
      if(msg.name=="PONG"):
        self.ac_downlink_status[sender].last_pong_time = time.time() - self.ac_downlink_status[sender].last_ping_time

  def start(self): 
    self.alive=True
    self.interface.start()
 
  def join(self,arg): 
    self.interface.join(arg)

  def stop(self):
    self.alive=False
    self.interface.stop()

  def send(self, msg, sender_id=None, receiver_id: int = 0, component_id=0, ack=False):
    if(self.alive):
      if receiver_id not in self.ac_downlink_status: 
        self.ac_downlink_status[receiver_id] = DownLinkStatus(receiver_id)
      self.interface.send(msg,sender_id,receiver_id,component_id)
      self.ac_downlink_status[receiver_id].tx_msgs += 1

  def sendPing(self,ping_time):
    if(self.alive):
      for ac_id in self.ac_downlink_status:
        self.interface.send(message.PprzMessage('datalink','PING'),0,ac_id,0)
        self.ac_downlink_status[ac_id].last_ping_time = ping_time


class CTRLink:

  def __init__(self,args):
    self.udp=False
    self.xbee=False
    self.args=args
    self.acs = []
    for ac in args.acs_conf:
      a = ac.split(':')
      if len(a) == 3 and args.address is not None:
        a.insert(1, args.address)
      elif len(a) == 3:
        a.insert(1, DEFAULT_ADDRESS)
      elif len(a) != 4:
        print("invalid number of AC conf parameters")
        exit()
      self.acs.append(a)

    if len(self.acs) != 0:  self.udp=True
    if(args.dev=="/dev/ttyUSB0")or(args.dev=="/dev/paparazzi/xbee")and(args.baud==57600)and(args.transp=="xbee"): 
      if(os.path.exists(args.dev)):self.xbee=True
      else: print("Device not connected %s !!" % args.dev)

    if(self.udp)or(self.xbee):
      messages_xml_map.parse_messages()
      self.ivy = pprzlink.ivy.IvyMessagesInterface("LinkPy", True, False, args.bus)
      self.link_comb=Link_Combiner(self.ivy)
      if(self.udp):self.udp_interface=UDPLink(self.link_comb,self.acs)
      if(self.xbee):self.xbee_interface=XBEELink(self.link_comb,args)
      self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
      self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)


  def run(self):
    try:
      self.ivy.start()

      if(self.xbee):self.xbee_interface.start()
      if(self.udp):self.udp_interface.start()

      self.ping_timer.start()
      self.status_timer.start()
      self.initial_ivy_binds()

#      while (((self.xbee)and(self.xbee_interface.is_alive()))or((self.udp)and(self.udp_interface.is_alive()))):
#          if(self.xbee):self.xbee_interface.join(1)
#          if(self.udp):self.udp_interface.join(1)

      if(self.xbee):
        while(True):
          self.xbee_interface.join(None)
          # Process USB reconnection
          self.xbee_interface.stop()
          while not (os.path.exists(self.args.dev)):time.sleep(1)
          self.xbee_interface=XBEELink(self.link_comb,self.args)
          self.xbee_interface.start()


    except (KeyboardInterrupt, SystemExit):
      print('Shutting down...')

      if(self.xbee):self.xbee_interface.stop()
      if(self.udp):self.udp_interface.stop()

      exit()
      
      

  def initial_ivy_binds(self):
    messages_datalink = messages_xml_map.get_msgs("datalink")
    for msg in messages_datalink:
      self.ivy.subscribe(self.proccess_uplink_msg, message.PprzMessage("datalink", msg))


  def sendPing(self):
    ping_time = time.time()
    self.ping_timer = threading.Timer(PING_PERIOD, self.sendPing)
    self.ping_timer.start()
    if(self.xbee):self.xbee_interface.sendPing(ping_time)
    if(self.udp):self.udp_interface.sendPing(ping_time)


  def sendStatus(self):
    self.status_timer = threading.Timer(STATUS_PERIOD, self.sendStatus)
    self.status_timer.start()
    single=False
    if(self.xbee):
      if not(self.udp):single=True
      self.sendStatusIvy(single,self.xbee_interface)
    if(self.udp):
      if not(self.xbee):single=True
      self.sendStatusIvy(single,self.udp_interface)


  def sendStatusIvy(self,single,interface):
    if(single):link=-1 
    else:link=interface.link_id
    downlink=interface.ac_downlink_status
    for item in downlink:
      value=downlink[item]
      self.ivy.send("link LINK_REPORT %i %i %i %i %i %i %i %i %i %i %i" % (
           value.ac_id,
           link,
           value.run_time,
           time.time() - value.last_msg_time,
           value.rx_bytes,
           value.rx_msgs,
           0,
           value.rx_bytes - value.last_rx_bytes,
           value.rx_msgs - value.last_rx_msgs,
           value.tx_msgs,
           1000 * value.last_pong_time))
      value.last_rx_bytes = value.rx_bytes
      value.last_rx_msgs = value.rx_msgs
      value.run_time = value.run_time + 1



  def proccess_uplink_msg(self,ac_id,msg):
    if(self.xbee):self.xbee_interface.send(msg,None,int(ac_id))
    if(self.udp):self.udp_interface.send(int(ac_id),msg)



if __name__ == '__main__':

  from argparse import ArgumentParser

  parser = ArgumentParser(description="XBEE/UDP link for paparazzi (python version)")

  group1 = parser.add_argument_group("group 2")
  group1.add_argument('-ac', dest='acs_conf', action='append', default=[], 
    help="AC configuration with format ID:[IP:]PORT_OUT:PORT_IN (multiple possible)")

  group2 = parser.add_argument_group("group 1")
  group2.add_argument("-d", "--device", help="device name", dest='dev')
  group2.add_argument("-s", "--baudrate", help="baudrate", dest='baud', type=int)
  group2.add_argument("-t", "--transport", help="pprz or xbee", dest='transp')
#  group2.add_argument("-id", "--ac_id", help="aircraft id (receiver)", dest='ac_id', default=0, type=int)


#    parser.add_argument("--broadcast_method", choices=[IP_BCAST,REPEAT] , default=IP_BCAST, help="Broadcast method - repeating to all known aircraft or sending to IP broadcast address. [default: %(default)s]")
#    parser.add_argument("--broadcast_address", default=DEFAULT_BROADCAST, help="IP address used for broadcast when broadcast method is IP_BCAST. [default: %(default)s]")
#    parser.add_argument("--uplink_port", default=pprzlink.udp.UPLINK_PORT, help="Uplink UDP port. [default: %(default)s]")
#    parser.add_argument("--downlink_port", default=pprzlink.udp.DOWNLINK_PORT, help="Downlink UDP port. [default: %(default)s]")

  parser.add_argument("--bus", default=pprzlink.ivy.IVY_BUS, help="Ivy bus. [default to system IVY bus]")

  args = parser.parse_args()

  try:
    link = CTRLink(args)
    if((link.xbee)or(link.udp)):link.run()
  except ValueError as e:
    print(e)
