#!/usr/bin/env python3

import sys
import wx
import argparse

def main():
    parser = argparse.ArgumentParser("gvfApp", description="Python App for visualization GVF while PaparazziUAV is running")
    parser.add_argument('id_aircraft',type=int,help="ID of the aircraft(s) one wants to plot its GVF and trajectory. \
                        If there are several, do simpler XY,YZ,XZ plots with all mentionned aircrafts with their trajectories (no GVF by default).\
                        Not available in 'legacy' mode.", 
                        nargs='+')
    parser.add_argument('-w','--w-dist', type=float, dest='w_dist',default=400,
                        help="Range around the current virtual parametric coordinate on which to plot the trajectory")
    parser.add_argument('-f','--show-field',action='store_true',dest='show_field',default=False,
                        help="Flag to force the display of the GVF when tracking multiple aircrafts.")
    parser.add_argument('--no-field',action='store_true',dest='no_field',default=False,
                        help="Flag to disable the disaply of the GVF when tracking one aircraft. Takes precedence over '--show-field'")
    parser.add_argument('-r','--resolution', type=int, dest='resolution',default=20,
                        help="Number of points per dimensions used to plot the GVF (and 10 times more for the trajectory)")
    parser.add_argument('-g','--gvf-dist', type=float,dest='gvf_dist',default=300,
                        help="L1 range around the aircraft on which to plot the GVF")
    parser.add_argument('-lt','--list-traj',action='store_true',dest='list_traj',default=False,
                        help="If set, list all known trajectories ('classic' GVF and parametric GVF)")
    
    
    
    parser.add_argument('--legacy',dest='legacy', action='store_true', default=False,
                        help="If set, uses the original 'gvfframe.py' script. Otherwise, default to 'modernGVFframe.py'."\
                            "Note that this disable all other optional arguments.")
    
    args = parser.parse_args()
    ac_id = args.id_aircraft
    
    if not args.legacy:
        from modernGVFMultiframe import GVFFrame
        
        if args.list_traj:
            from modernGVFframe import GVF_traj_map,GVF_PARAMETRIC_traj_map
            print("Known 'classic' GVF trajectories")
            for id,c in GVF_traj_map.items():
                print(f"  - ID {id:2} : {c.__name__}")
                
            print("\nKnown parametric GVF trajectories")
            for id,c in GVF_PARAMETRIC_traj_map.items():
                print(f"  - ID {id:2} : {c.__name__}")
            print('\n')
        
        
        class MessagesApp(wx.App):
            def __init__(self, wtf, ac_ids, gvf_kwargs):
                self.ac_ids = ac_ids
                self.gvf_kwargs = gvf_kwargs
                wx.App.__init__(self, wtf)

            def OnInit(self):
                self.main = GVFFrame(self.ac_ids,**self.gvf_kwargs)

                self.main.Show()
                self.SetTopWindow(self.main)
                return True
            
        
    else:
        if len(ac_id) > 1:
            print(f"Given arguments: {args}",file=sys.stderr)
            raise ValueError("Cannot plot several aircrafts at once in 'legacy' mode.")
        
        from gvfframe import GVFFrame 
        class MessagesApp(wx.App):
            def __init__(self, wtf, ac_id, gvf_kwargs:dict=dict()):
                self.ac_id = ac_id[0]
                wx.App.__init__(self, wtf)

            def OnInit(self):
                self.main = GVFFrame(self.ac_id)

                self.main.Show()
                self.SetTopWindow(self.main)
                return True
    
    gvf_kwargs = dict()
    gvf_kwargs['w_dist'] = float(args.w_dist)
    gvf_kwargs['resolution'] = int(args.resolution)
    gvf_kwargs['gvf_dist'] = float(args.gvf_dist)
    if args.no_field:
        gvf_kwargs['show_field'] = False
    elif args.show_field:
        gvf_kwargs['show_field'] = True
    else:
        gvf_kwargs['show_field'] = len(ac_id) < 2
 
    application = MessagesApp(0, ac_id, gvf_kwargs=gvf_kwargs)
    application.MainLoop()

if __name__ == '__main__':
    main()
