#!/usr/bin/env python3

import wx
import argparse

def main():
    parser = argparse.ArgumentParser("gvfApp", description="Python App for visualization GVF while PaparazziUAV is running")
    parser.add_argument('id_aircraft',type=int,help="ID of the aircraft one wants to plot its GVF")
    parser.add_argument('-d','--dim',dest='dim',choices=['2','3'],default=2,
                        help="Dimension asked for plotting (either 2 or 3, default is 2)")
    parser.add_argument('-w','--w-dist', type=float, dest='w_dist',default=400,
                        help="Range around the current virtual parametric coordinate on which to plot the trajectory")
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
    id_ac = args.id_aircraft
    
    if not args.legacy:
        from modernGVFframe import GVFFrame
        
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
            def __init__(self, wtf, ac_id, gvf_kwargs):
                self.ac_id = ac_id
                self.gvf_kwargs = gvf_kwargs
                wx.App.__init__(self, wtf)

            def OnInit(self):
                self.main = GVFFrame(self.ac_id,**self.gvf_kwargs)

                self.main.Show()
                self.SetTopWindow(self.main)
                return True
        
    else:
        from gvfframe import GVFFrame 
        class MessagesApp(wx.App):
            def __init__(self, wtf, ac_id, gvf_kwargs:dict=dict()):
                self.ac_id = ac_id
                wx.App.__init__(self, wtf)

            def OnInit(self):
                self.main = GVFFrame(self.ac_id)

                self.main.Show()
                self.SetTopWindow(self.main)
                return True
    
    gvf_kwargs = dict()
    gvf_kwargs['dim'] = int(args.dim)
    gvf_kwargs['w_dist'] = float(args.w_dist)
    gvf_kwargs['resolution'] = int(args.resolution)
    gvf_kwargs['gvf_dist'] = float(args.gvf_dist)
    
    application = MessagesApp(0, id_ac, gvf_kwargs=gvf_kwargs)
    application.MainLoop()

if __name__ == '__main__':
    main()
