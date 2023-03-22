#!/usr/bin/env python3
#
# Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

"""
Connect to paparazzi server to get the aircraft list and configurations

See http://wiki.paparazziuav.org/wiki/DevGuide/Server_GCS_com for more details

:Example:

    import pprz_connect
    import time

    # define a callack
    def new_ac(conf):
        print(conf)

    connect = PprzConnect(notify=new_ac)

    # do some things here or wait for event
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    # close before leaving
    connect.shutdown()

"""

from __future__ import print_function
import sys
from os import path, getenv, getpid
from time import sleep
import typing
import dataclasses

try:
    from pprzlink.ivy import IvyMessagesInterface
    from pprzlink.message import PprzMessage
    pprzImport = True
except ImportError:
    pprzImport = False

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

if not(pprzImport):
    from pprzlink.ivy import IvyMessagesInterface
    from pprzlink.message import PprzMessage

from ivy.std_api import *

@dataclasses.dataclass
class PprzConfig(object):
    """
    Aircraft configuration as sent by the Paparazzi server.
    Getter/setter functions should be used to access the attributes.
    """

    ac_id:int
    ac_name:str
    airframe:str
    flight_plan:str
    settings:str
    radio:str
    color:str
    
    def __post_init__(self) -> None:
        self.ac_id = int(self.ac_id)

    @property
    def id(self) -> int:
        return self.ac_id 

    @property
    def name(self) -> str:
        return self.ac_name    


class PprzConnect(object):
    """
    Main class to handle the initialization process with the server
    in order to retrieve the configuration of the known aircraft
    and update for the new ones
    """

    def __init__(self, notify:typing.Optional[typing.Callable[[PprzConfig],None]]=None, ivy:typing.Optional[IvyMessagesInterface]=None, verbose:bool=False):
        """
        Init function
        Create an ivy interface if not provided and request for all aircraft

        :param notify: callback function called on new aircraft, takes a PprzConfig as parameter
        :param ivy: ivy interface to contact the server, if None a new one will be created
        :param verbose: display debug information
        """
        self.verbose = verbose
        self._notify = notify

        self._conf_list_by_name:typing.Dict[str,PprzConfig] = {}
        self._conf_list_by_id:typing.Dict[int,PprzConfig] = {}

        if ivy is None:
            self._ivy = IvyMessagesInterface("PprzConnect")
        else:
            self._ivy = ivy
        sleep(0.1)

        self.get_aircrafts()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        """
        Shutdown function

        Should be called before leaving if the ivy interface is not closed elsewhere
        """
        if self._ivy is not None:
            if self.verbose:
                print("Shutting down ivy interface...")
            self._ivy.shutdown()
            self._ivy = None

    def conf_by_name(self, ac_name=None):
        """
        Get a conf by its name

        :param ac_name: aircraft name, if None the complete dict is returned
        :type ac_name: str
        """
        if ac_name is not None:
            return self._conf_list_by_name[ac_name]
        else:
            return self._conf_list_by_name

    def conf_by_id(self, ac_id=None):
        """
        Get a conf by its ID

        :param ac_id: aircraft id, if None the complete dict is returned
        :type ac_id: str
        """
        if ac_id is not None:
            return self._conf_list_by_id[ac_id]
        else:
            return self._conf_list_by_id

    @property
    def ivy(self) -> IvyMessagesInterface:
        """
        Getter function for the ivy interface
        """
        return self._ivy

    def get_aircrafts(self):
        """
        request all aircrafts IDs from a runing server
        and new aircraft when they appear
        """
        def aircrafts_cb(sender, msg):
            ac_list = msg['ac_list']
            for ac_id in ac_list:
                self.get_config(ac_id)
            #ac_list = [int(a) for a in msg['ac_list'].split(',') if a]
            if self.verbose:
                print("aircrafts: {}".format(ac_list))
        self._ivy.send_request('ground', "AIRCRAFTS", aircrafts_cb)

        def new_ac_cb(sender, msg):
            ac_id = msg['ac_id']
            self.get_config(ac_id)
            if self.verbose:
                print("new aircraft: {}".format(ac_id))
        self._ivy.subscribe(new_ac_cb,PprzMessage('ground','NEW_AIRCRAFT'))

    def get_config(self, ac_id:str):
        """
        Request a config from the server for a given ID

        :param ac_id: aircraft ID
        :type ac_id: str
        """
        def conf_cb(sender, msg):
            conf = PprzConfig(msg['ac_id'], msg['ac_name'], msg['airframe'],
                    msg['flight_plan'], msg['settings'], msg['radio'],
                    msg['default_gui_color'])
            self._conf_list_by_name[conf.name] = conf
            self._conf_list_by_id[conf.id] = conf
            if self._notify is not None:
                self._notify(conf) # user defined general callback
            if self.verbose:
                print(conf)
        self._ivy.send_request('ground', "CONFIG", conf_cb, ac_id=ac_id)


if __name__ == '__main__':
    """
    test program
    """
    try:
        connect = PprzConnect(verbose=True)
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("Stopping on request")

    connect.shutdown()

