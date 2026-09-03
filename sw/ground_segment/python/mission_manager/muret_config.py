import os,asyncio,time,datetime,pickle,pathlib

import numpy as np
import matplotlib.pyplot as plt

from fleet_manager import FleetManager,FleetManagerEnd,pyproj,ACStats,Pose3D
from uav_data import LandingSite
import Formation as formation
from FleetPath import formation_oval,formation_rectangle

from tracking_logs_analyser import plot_trackingdata

##### General setup #####
color_dict = {
        60 : 'blue',
        61 : 'black', # Use a legible colors for graph... # 61 : 'white',
        62 : 'red',
        63 : 'green'
    }
    
transformer = pyproj.Transformer.from_crs('WGS84','EPSG:9794') # Default to WGS84 to Lambert93
home_lat = 43.4626512
home_lon = 1.2732883
home_height = 225-185
home_alt = 225
home_x,home_y = transformer.transform(home_lat,home_lon)
stat_list = [ACStats(i,14.1,10/60,40,(i%10)*10+10+home_alt) for i in [60,61,62]]
separation = 42

##### Demo formation #####
frmtion = formation.chevron_formation(len(stat_list),separation*4/3).reorder([1,2,0])
    
start = Pose3D(home_x,home_y+50,home_alt,0.)
fleet_plan = formation_oval(stat_list,start,80,60,frmtion)
# fleet_plan = formation_rectangle(stat_list,start,300,100,formation)
fleet_plan.keyposes = [fleet_plan.keyposes[0],fleet_plan.keyposes[2]]

# Adapt altitude using the reference cruise one 
for p in fleet_plan.keyposes:
    for i in range(len(stat_list)):
        p[i,2] = stat_list[i].cruise_altitude
    
##### Landing formation #####
preland_frmtion = formation.column_formation(len(stat_list),70).reorder([2,1,0])
preland_frmtion.orientation -= np.deg2rad(136)
preland_frmtion.center = np.array([home_x+130,home_y+110,home_alt])
land_to_af_direction = 109.
land_to_af_bisdirection = land_to_af_direction + 180.
land_to_af_distance = 160.
possible_afs = [(land_to_af_direction,land_to_af_distance),(land_to_af_bisdirection,land_to_af_distance)]

landing_sites = [LandingSite(43.4626595, 1.2732809, 0., possible_afs, 70),
                 LandingSite(43.4627104, 1.2733237, 0., possible_afs, 70),
                 LandingSite(43.4627618, 1.2733660, 0., possible_afs, 70)] 



if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('fleet_plan',help='Plan type',choices=['demo','land','landing','actual'])
    parser.add_argument('dubins_solver',help='Path to Dubins Fleet Planner')
    parser.add_argument('--verbose', '-v', action='count', default=0)
    parser.add_argument('-t','--takeoff',action='store_true',help='Send take off mission order first')
    parser.add_argument('--ignore-wind',action='store_true',dest='ignore_wind',help='If set, ignore wind information from aircraft')
    parser.add_argument('--speed-ctl',action='store_true',dest='speed_ctl',help='If set, enable speed control for aircraft')
    parser.add_argument('--autostart',type=float,help='If set, automatically launch the main after the given value (in seconds). Otherwise, wait for user input to start the main loop',
                                default=None)
    parser.add_argument('--carrot',type=float, help='Anticipation time (in seconds) for declaring the current plan end. Default to 5s',
                                default=5)
    parser.add_argument('-G','--obstacles',type=str,help="Path to the file describing the obstacles (as lines and circles).",default=None)
    
    args = parser.parse_args()
        
    if args.carrot < 0.:
        raise ValueError(f"--carrot argument must be non-negative! Current value is: {args.carrot}")
    
    request_land = False
    
    if args.fleet_plan == 'actual':
        fleet_plan.keyposes.append(preland_frmtion.get_abs_positions())
        request_land = True
    
    if args.fleet_plan[0:4] == 'land':
        fleet_plan.keyposes = [preland_frmtion.get_abs_positions()]
        request_land = True
    
    obstacles_path = None
    if args.obstacles is not None:
        obstacles_path = pathlib.Path(args.obstacles)
        if not(obstacles_path.is_file()):
            print(f"The given paths for obstacles does not point to a file:\n{args.obstacles}")
            exit(1)
        
    manager = FleetManager(
        fleet_plan,
        args.dubins_solver,
        separation,
        home_alt,
        geometric_obstacles_path=obstacles_path,
        transformer=transformer,
        end_strategy=FleetManagerEnd.LAND if request_land else FleetManagerEnd.NOTHING,
        speed_ctl=args.speed_ctl,
        ignore_wind=args.ignore_wind,
        verbosity=args.verbose,
        tracking_error=20
    )
    manager.samples = 0 #if obstacles_path is None else 10
    manager.end_of_plan_carrot = args.carrot
    manager.extra_straight_length = 60
    manager.nps_simulation = False
    
    if args.fleet_plan[0:4] == 'land':
        manager.landing_sites = landing_sites
        
    main_loop_exception = None
        
    try:
        manager.wait_ready(15)
        if args.takeoff:
            print("Take off requested!")
            manager.takeoff(home_height)
        # manager.circle_home(insert_mode=MissionInsert.APPEND)
        # manager.start_mission()
        if args.autostart is not None:
            print(f"Wait for {args.autostart} seconds before starting main loop...",end=' ',flush=True)
            time.sleep(args.autostart)
            print("Starting now!")
        else:
            input("Press any key to start fleet path planning...")
        asyncio.run(manager.main())
    except(KeyboardInterrupt,SystemExit):
        print("Intettupted!")
    except Exception as e:
        print("Some exception interrupted the main loop...:\n",e)
        main_loop_exception = e
        raise e
    finally:
        print("Closing")
        manager.closing()
        
        print("Saving logs")
        logs = manager.get_logs()
        
        os.makedirs("logs",exist_ok=True)
        now_str = datetime.datetime.fromtimestamp(time.time()).strftime('%y-%m-%d_%H:%M:%S')
        filename = f"logs/logs_{now_str}.pkl"
        with open(filename,"wb") as f:
            pickle.dump(logs, f)
        
        try:
            print("Plotting logs...")
            fig,traj_ax,axes,selectors = plot_trackingdata(logs,color_dict,obstacles_path)
            fig.set_size_inches(16,9)
            fig.tight_layout()
            plt.show()
        except KeyError as e:
            # If there is a key error, the log is incomplete, so delete it
            os.remove(f"logs/logs_{now_str}.pkl")
            print("KeyError while plotting logs, deleted the log file. Error: ",e)
            raise e
            
            
        if main_loop_exception is not None:
            raise main_loop_exception
        
        
        