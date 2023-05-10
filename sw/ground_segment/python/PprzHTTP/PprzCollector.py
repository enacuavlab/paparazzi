#!/usr/bin/env python3

import typing
from collections import deque

import sys
from time import sleep
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

import pprzlink.ivy
from pprzlink.ivy import IvyMessagesInterface,IVY_BUS
from pprzlink.message import PprzMessage

from pprz_connect import PprzConnect,PprzConfig



class PprzCollector():
    """
    Listen to an Ivy bus and store what passes.
    Made for Paparazzi messages, but could be adjusted for any type of messages...
    """
    
    def __init__(self,
                 messages:typing.Collection[str] = None,
                 size:int = 50, 
                 agent_name:str="PprzCollector",
                 ivy_bus:str=IVY_BUS,
                 verbosity:int=0):
        """
        Constructor for a PprzCollector

        Args:
            messages (typing.Collection[str], optional): Collection of messages to surveil. If None, surveil ALL messages. Defaults to None.
            size (int, optional): Size of the queues storing the messages (one per message type). Defaults to 50.
            agent_name (str, optional): Name of the agent on the Ivy bus. Defaults to "PprzCollector".
            ivy_bus (str, optional): Name of the Ivy bus to connect. Defaults to IVY_BUS.
            verbosity (int, optional): Verbosity level:
                - At 0, nothing is logged
                - At 1, PprzConnect logging is activated (info about new aicrafts)
                - At 2, PprzConnect and IvyMessagesInterface are activated (all messages captured)
                Defaults to 0.
        """
        
        self._message_collection:typing.Dict[str,deque[PprzMessage]] = dict()
        if messages:
            for s in messages:
                self._message_collection[s] = deque(maxlen=size)
        else:
            try:
                for s in pprzlink.ivy.msg_str_to_class.keys():
                    self._message_collection[s] = deque(maxlen=size)
            except Exception as e:
                print(f"Could not initialize the message collection statically:\n{e}")
                self._message_collection = dict()
                
        self.__ivy = IvyMessagesInterface(agent_name=agent_name,
                                          verbose=verbosity>1,
                                          ivy_bus=ivy_bus)
        
        def ivy_messasge_cb(ac_id:int,msg:PprzMessage) -> None:
            if messages:
                if msg.name in self._message_collection.keys():
                    self._message_collection[msg.name].append(msg)
            else:
                try:
                    self._message_collection[msg.name].append(msg)
                except KeyError:
                    self._message_collection[msg.name] = deque([msg],maxlen=size)
            
        
        self.__ivy.subscribe(ivy_messasge_cb,'(.*)' )
        
        self._ac_collection:typing.Dict[int,PprzConfig] = dict()
        self._ac_name_to_id:typing.Dict[int,str] = dict()
        
        def new_aircraft_cb(c:PprzConfig) -> None:
            self._ac_collection[c.ac_id] = c
            self._ac_name_to_id[c.ac_name] = c.ac_id
        
        self.__pprz_connect = PprzConnect(new_aircraft_cb,
                                          self.__ivy,
                                          verbose=verbosity>0)
        
    def get_messages(self,name:str,nbr:int=0) -> list[PprzMessage]:
        """
        Return the list of messages with the given 'name' collected on the Ivy bus

        If 'nbr' is 0, return all known messages, ordered by oldest first
        If 'nbr' is positive, return at most 'nbr' messages, ordered by oldest first
        If 'nbr' is negative, return at most '-nbr' messages, ordered by newest first
        """
        if nbr == 0:
            return list(self._message_collection[name])
        else:
            if nbr > 0:
                return list(m for (m,i) in zip(self._message_collection[name],range(nbr)))
            else:
                return list(m for (m,i) in zip(reversed(self._message_collection[name],range(nbr))))
            
    def pop_messages(self,name:str,nbr:int=0) -> list[PprzMessage]:
        """
        Return the list of messages with the given 'name' collected on the Ivy bus, and remove them from the internal queue

        If 'nbr' is 0, return all known messages, ordered by oldest first
        If 'nbr' is positive, return at most 'nbr' messages, ordered by oldest first
        If 'nbr' is negative, return at most '-nbr' messages, ordered by newest first
        """
        queue_size = len(self._message_collection[name])
        if nbr == 0:
            return list(self._message_collection[name].popleft() for i in range(queue_size))
        else:
            if nbr > 0:
                return list(self._message_collection[name].popleft() for i in range(min(nbr,queue_size)))
            else:
                return list(self._message_collection[name].pop() for i in range(min(-nbr,queue_size)))
            
    def get_ac_info(self,id:typing.Union[str,int]) -> PprzConfig:
        """
        Given an identifier (either the Aicraft name, as a string, or its id, as an int), returns its config
        """
        if isinstance(id,str):
            ac_id = self._ac_name_to_id[id]
        else:
            ac_id = id
        return self._ac_collection[ac_id]
    
    def get_all_ac(self) -> typing.Dict[int,PprzConfig]:
        """
        Returns a copy of the dictionary storing the aircrafts' info (indexed by aircraft id)
        """
        return self._ac_collection.copy()
   
    
    def __del__(self):
        self.__pprz_connect.shutdown()
        self.__ivy.shutdown()


def main():
    collector = PprzCollector(verbosity=2)
    
    while True:
        try:
            sleep(1.)
            print(collector.get_all_ac())
        except KeyboardInterrupt:
            del collector
            return
        

if __name__ == "__main__":
    main()