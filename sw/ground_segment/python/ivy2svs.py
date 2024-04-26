#!/usr/bin/env python

import threading, sys, signal, os
import logging
from ivy import ivy 
import math

from time import sleep
import argparse

#DEFAULTBUS = "224.255.255.255:2010"
#DEFAULTBUS="127.255.255.255:2010"
#DEFAULTBUS=""

format_TO_ATC="^(\\S+) TO_ATC (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+)"

fieldNames = ["Flight=", "CallSign=", "Lat=", "Lon=", "Alt=", "Track=", "Speed=", "VSpeed=", "Heading=", "Pitch=", "Roll=", "TACode=", "Company=", "Model="]

def on_TO_ATC(agent, *larg):
  for element in dictlist:
   if element['ac_id'] == int(larg[0]):

     res = "DroneEvent"
     for i, name in enumerate(fieldNames):
       res += " " + name + larg[i+1].replace("\"", '')
     bus.send_msg(res)
 
def on_Period(IvyTimer):
  bus.send_msg("Alive")



if __name__ == '__main__':

  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-ac', metavar='ac', nargs='+', type=int, help='A/C id (multiple possible)')
  parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
  args = parser.parse_args()

  if args.ac is None:
    print("At least one AC id must be declared")
    exit()

  dictlist = []
  for line in args.ac:
    dictlist.append({'ac_id': line,
                       'str' : ''})

  print(dictlist)

  try:
    bus = ivy.IvyServer('Ivy2svs', 'Ivy2svs READY')
    ivy.ivylogger.setLevel(logging.ERROR)

    bus.bind_msg(on_TO_ATC, format_TO_ATC)

    timerid = ivy.IvyTimer(bus,0,1000,on_Period)
    bus.start(args.ivy_bus)
    timerid.start()

  except (KeyboardInterrupt, SystemExit):
    timerid.stop()
    ivy.shutdown()
  except OSError:
    timerid.stop()
    ivy.stop()
    exit(-1)
