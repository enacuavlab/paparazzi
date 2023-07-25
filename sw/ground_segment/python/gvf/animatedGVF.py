#!/usr/bin/env python3

import sys
import argparse

from animatedGVF.animation import Trajectory3DMap

def main():
    parser = argparse.ArgumentParser("AnimatedGVF", description="A Matplotlib based dynamic visualisation tool\
        for GVF-based PaparazziUAV flights. Show a 3D plot and the 3 'canonical' projections (XY,XZ,YZ).")
    
    parser.add_argument('ids',nargs='+',
                        help="List of aircraft ids' to track")
    parser.add_argument('--name',type=str,dest='name',default='GVF_collector',
                        help="Name for the underlying Ivy interface collecting messages. Default is 'GVF_collector'")
    parser.add_argument('-w','--w-dist',dest='w_dist',type=float,default=400,
                        help="Range (positive) around the carrot point for plotting the trajectory.\
                            May be ignored for closed trajectory (plot everything instead).\
                            For non-parametric ones (no carrot), some arbitrary reference point is used...\
                            Default is 400.")
    
    parser.add_argument('-gvf',dest='gvf',default=False,nargs='?',
                        help="Display the Guiding Vector field around the aircrafts (only in the planar projections).\
                        With no arguments, enable it with range 300m. Otherwise, enable it with specified range.\
                        By default: enabled if only one aircraft is tracked, disabled otherwise.")
    
    parser.add_argument('--no-gvf',dest='no_gvf',action='store_true',default=False,
                        help="Disable GVF plotting entierly. Superseed the 'gvf' flag.")
    
    parser.add_argument('-r','--resolution',dest='resolution',default=10,type=int,
                        help="Number of datapoints per dimension used when computing the GVF.\
                        For computing trajectories, the number of datapoints is 10 times the resolution.\
                        Default is 10.")
    
    parser.add_argument('-hs','--history-size',dest='history_size',type=int,default=100,
                        help="Number of previous positions shown (in frames) kept while plotting.\
                        If set to 0 or 1, no history is shown. Default is 100.")
    
    parser.add_argument('-fps',dest='fps',type=int,default=30,
                        help="Number of frames per second wanted for the animation\
                        (i.e. number of look-up of the current state per second, the state being automatically updated\
                        by the Ivy interface manager.) Default is 30.")
    
    parser.add_argument('-n','--normalized',dest='normalized',default=False,action='store_true',
                        help="Consider the GVF normalizing the parametric curve (follows g(t) = f(s(t)) such that ||g(t)|| = 1)")
    
    args = parser.parse_args()
    
    ac_ids = [int(i) for i in args.ids]
    if args.no_gvf:
        show_field = False
        gvf_dist = 300.
    else:
        if args.gvf is False:
            show_field = len(ac_ids) == 1
        else:
            show_field = True
        
        if args.gvf is not None and args.gvf is not False:
            gvf_dist = float(args.gvf)
        else:
            gvf_dist = 300.
    
    map = Trajectory3DMap(ac_ids,args.name,args.w_dist,show_field,gvf_dist,
                          args.resolution,args.history_size,args.fps,args.normalized)
    
    map.start()
    
if __name__ == "__main__":
    main()
            
    
    
    
    