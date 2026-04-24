#!/usr/bin/env python3
#
# Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#

from cohoma_bridge import MissionManager, MissionInsert

LANDPAD = (43.3153917, 1.4041498)
P1 = (43.3153875, 1.4035381)
P2 = (43.3159358, 1.4031349)
P3 = (43.3161746, 1.4046671)

try:
    mission = MissionManager(verbose=True)
    print("cohoma_bridge test started")
    mission.wait_ready()
    mission.add_mission_takeoff(1, height=40., insert_mode=MissionInsert.REPLACE_ALL)
    mission.add_mission_path(2, path=[LANDPAD, P1, P2], alt=350)
    mission.add_mission_poles(3, lat1=P2[0], lon1=P2[1], lat2=P3[0], lon2=P3[1], height=40., radius=60., nb_laps=2)
    #mission.add_mission_path(4, path=[P2, P1, LANDPAD] , alt=350)
    mission.add_mission_land(5, lat=LANDPAD[0], lon=LANDPAD[1], height=0.)
    mission.start_mission()
    print('UAV:',mission.uav_data.name)

except(KeyboardInterrupt,SystemExit):
    print("interrupt cohoma_bridge test")
    mission.closing()
finally:
    print("closing cohoma_bridge test")
    mission.closing()
