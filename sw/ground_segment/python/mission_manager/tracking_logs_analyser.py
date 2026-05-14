#!/usr/bin/env python3

import pickle
import pathlib
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from Dubins import Pose3D,poses_XY_dist,min_XY_dist,ACStats
from plotting import DictOfPoseTrajectories,plot_several_pose2d_sequences
from uav_data import UAVData, TrackingLogs

def plot_trackingdata(logs: TrackingLogs, color_dict: dict[int,str], 
                      hide_refs:bool=False, 
                      hide_reschedules:bool=False,
                      hide_keyframes:bool=False) -> tuple[Figure,Axes,Axes,Axes]:
    trajs = logs.as_trajectories()
    refs = logs.ref_trajectories()
    separations = logs.get_all_min_sep()
    errs = logs.get_errors()
    replanning_timestamps = logs.replanning_timestamps
    stat_list = logs.ac_stats
    ## Plotting
    fig,axes = plt.subplots(2,2,
                            sharex=True,sharey=False,
                            width_ratios=[2,1],)
    
    # Big ax for trajectories
    gs = axes[0,0].get_gridspec()
    axes[0,0].remove()
    axes[1,0].remove()
    
    traj_ax = fig.add_subplot(gs[:,0])
    
    traj_ax.set_aspect('equal')
    
    if not hide_keyframes:
        keyframe_poses = logs.keyframes.keyposes
        for pose_set in keyframe_poses:
            xs = pose_set[:,0]
            ys = pose_set[:,1]
            dxs = np.cos(pose_set[:,3])
            dys = np.sin(pose_set[:,3])
            traj_ax.quiver(xs,ys,dxs,dys,color='k',
                           angles='xy',)
            
    
    plot_several_pose2d_sequences(traj_ax,trajs,color_dict,
        label=True,endpoints=True)
    
    
    if not hide_refs:
        _,d = plot_several_pose2d_sequences(traj_ax,refs,color_dict,
            label=True,endpoints=True,linestyle='dashed')
        for e in d.values():
            l = e.get_label()
            e.set_label(l + " (ref)")
    
    
    if not hide_reschedules:
        # Adding rescheduling scatters
        actual_replanning_poses :DictOfPoseTrajectories = dict()
        ref_replanning_poses    :DictOfPoseTrajectories = dict()
        for stat in stat_list:
            id = stat.id
            r_interp = logs.ref_interpolator(id)
            a_interp = logs.traj_interpolator(id)
            r_poses = []
            a_poses = []
            for t in replanning_timestamps:
                r_poses.append((t,r_interp(t)))
                a_poses.append((t,a_interp(t)))
            actual_replanning_poses[id] = a_poses
            ref_replanning_poses[id] = r_poses

        _,d1 =plot_several_pose2d_sequences(traj_ax,actual_replanning_poses,color_dict,
            label=True,endpoints=False,linestyle=' ',marker="P",markersize=10)
        first = True
        for e in d1.values():
            if first:
                l = e.get_label()
                e.set_label(l + " (reschedule)")
                first = False
            else:
                e.set_label("")
    
        if not hide_refs:
            _,d2 = plot_several_pose2d_sequences(traj_ax,ref_replanning_poses,color_dict,
                label=True,endpoints=False,linestyle=' ',marker=(5,1,0),markersize=10)
            first = True
            for e in d2.values():
                if first:
                    l = e.get_label()
                    e.set_label(l + " (ref reschedule)")
                    first = False
                else:
                    e.set_label("")
        
        traj_ax.set_xlabel("Easting (m)")
        traj_ax.set_ylabel("Northing (m)")
        traj_ax.legend()
        
    # Smaller axes for min distance and tracking errors
    
    tracking_ax:Axes = axes[0,1]
    min_t = np.inf
    max_t = -np.inf
    max_val = -np.inf
    
    avg_err = 0
    for id,data in errs.items():
        ts = [t[0] for t in data]
        min_t = min(min_t,min(ts))
        max_t = max(max_t,max(ts))
        vals = [t[1] for t in data]
        avg_err += np.mean(vals)
        max_val = max(max_val,max(vals))
        tracking_ax.plot(ts,vals,label=f"{id}",color=color_dict[id])
    
    avg_err /= len(errs)
    tracking_ax.hlines(logs.tracking_error_threshold,min_t,max_t,colors='k',linestyles='dashed',label=f'Tracking error threshold: {logs.tracking_error_threshold:.1f} m')
    tracking_ax.vlines(replanning_timestamps,0.,max_val,colors='grey',linestyles='dotted',label='Effective replanning')
    tracking_ax.set_xlabel("Time (s)")
    tracking_ax.set_ylabel("Tracking error (m)")
    tracking_ax.set_title(f"Tracking error (avg: {avg_err:.1f} m)")
    tracking_ax.grid(True,'both','y')
    tracking_ax.legend()
    
    sep_ax:Axes = axes[1,1]
    ts = [t[0] for t in separations]
    vals = [t[1] for t in separations]
    sep_ax.plot(ts,vals)
    sep_ax.set_ylim(0.,100)
    sep_ax.hlines(logs.separation_threshold,ts[0],ts[-1],colors='k',linestyles='dashed',label=f'Required Separation: {logs.separation_threshold:.1f} m')
    sep_ax.vlines(replanning_timestamps,0.,max(vals),colors='grey',linestyles='dotted',label='Effective replanning')
    sep_ax.set_xlabel("Time (s)")
    sep_ax.set_ylabel("Minimum Separation (m)")
    sep_ax.grid(True,'both','y')
    sep_ax.legend()
    
    return fig,traj_ax,tracking_ax,sep_ax
    
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyse tracking logs")
    parser.add_argument("logfile",type=pathlib.Path,help="Path to the log file to analyse")
    parser.add_argument("--refs",action='store_true',help="Show the reference trajectories in the plot (useful when they are too cluttered)")
    parser.add_argument("--reschedules",action='store_true',help="Show the rescheduling points in the plot")
    parser.add_argument("--keyframes",action='store_true',help="Show the keyframe poses in the plot")
    args = parser.parse_args()
    
    with open(args.logfile,"rb") as f:
        logs:TrackingLogs = pickle.load(f)
        assert isinstance(logs,TrackingLogs), "The provided log file does not contain a TrackingLogs object"
    
    colorlist = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    color_dict = {}
    for i,stat in enumerate(logs.ac_stats):
        color_dict[stat.id] = colorlist[i%len(colorlist)]
        
    fig,traj_ax,tracking_ax,sep_ax = plot_trackingdata(logs,color_dict,
                                                       not(args.refs),
                                                       not(args.reschedules),
                                                       not(args.keyframes))
    
    fig.set_size_inches(16,9)
    fig.tight_layout()
    plt.show()