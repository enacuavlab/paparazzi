#!/usr/bin/env python3

import pickle
import pathlib
import argparse
import json

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.widgets import SpanSelector
from matplotlib.patches import Rectangle
from matplotlib.collections import PathCollection
from matplotlib.quiver import Quiver
from matplotlib.lines import Line2D 

from plotting import DictOfPoseTrajectories,plot_several_pose2d_sequences,plot_BasicPath_obstacle
from uav_data import UAVData, TrackingLogs
from ioUtils import parse_section_from_dict

def plot_trackingdata(logs: TrackingLogs, color_dict: dict[int,str], 
                      obstacles_path:Optional[pathlib.Path]=None,
                      hide_refs:bool=False, 
                      hide_reschedules:bool=False,
                      hide_keyframes:bool=False) -> tuple[Figure,Axes,list[Axes],list[SpanSelector]]:
    ## Plotting
    fig,axes = plt.subplots(3,2,
                            sharex=True,sharey=False,
                            width_ratios=[2,1],)
    
    # Big ax for trajectories
    gs = axes[0,0].get_gridspec()
    axes[0,0].remove()
    axes[1,0].remove()
    axes[2,0].remove()
    
    traj_ax = fig.add_subplot(gs[:,0])
    
    if obstacles_path is not None:
        with open(obstacles_path) as f:
            for e in json.load(f)["sections"]:
                plot_BasicPath_obstacle(traj_ax,parse_section_from_dict(e))
    
    _plot_trajs(traj_ax, logs, color_dict, hide_refs, hide_reschedules, hide_keyframes, labels=False, alpha=0.1)
    
    cover_lines = _plot_trajs(traj_ax, logs, color_dict, hide_refs, hide_reschedules, hide_keyframes)
        
    # Smaller axes for min distance and tracking errors
    
    tracking_ax:Axes = axes[0,1]
    _plot_tracking(tracking_ax, logs, color_dict)
    
    speed_ax:Axes = axes[1,1]
    _plot_speed(speed_ax,logs,color_dict)
    
    sep_ax:Axes = axes[2,1]
    _plot_separation(sep_ax, logs)
    
    time_lims = sep_ax.get_xlim()
    
    track_l_rect:Optional[Rectangle] = None
    track_r_rect:Optional[Rectangle] = None
    
    speed_l_rect:Optional[Rectangle] = None
    speed_r_rect:Optional[Rectangle] = None
    
    sep_l_rect:Optional[Rectangle] = None
    sep_r_rect:Optional[Rectangle] = None
    
    def onselect(tmin,tmax):
        nonlocal track_l_rect, track_r_rect, speed_l_rect, speed_r_rect, sep_l_rect, sep_r_rect, cover_lines
        
        if track_l_rect is not None:
            track_l_rect.remove()
        if track_r_rect is not None:
            track_r_rect.remove()
            
        if speed_l_rect is not None:
            speed_l_rect.remove()
        if speed_r_rect is not None:
            speed_r_rect.remove()
            
        if sep_l_rect is not None:
            sep_l_rect.remove()
        if sep_r_rect is not None:
            sep_r_rect.remove()
        
        track_l_rect = tracking_ax.axvspan(time_lims[0],tmin,color='grey',alpha=0.3)
        track_r_rect = tracking_ax.axvspan(tmax,time_lims[1],color='grey',alpha=0.3)
        tracking_ax.set_xlim(time_lims)
        
        speed_l_rect = speed_ax.axvspan(time_lims[0],tmin,color='grey',alpha=0.3)
        speed_r_rect = speed_ax.axvspan(tmax,time_lims[1],color='grey',alpha=0.3)
        speed_ax.set_xlim(time_lims)
        
        sep_l_rect = sep_ax.axvspan(time_lims[0],tmin,color='grey',alpha=0.3)
        sep_r_rect = sep_ax.axvspan(tmax,time_lims[1],color='grey',alpha=0.3)
        sep_ax.set_xlim(time_lims)
        
        for l in cover_lines:
            l[0].remove()
            for e in l[1]:
                e.remove()
            for e in l[2]:
                e.remove()
        cover_lines.clear()
        
        cover_lines = _plot_trajs(traj_ax, logs, color_dict, hide_refs, hide_reschedules, hide_keyframes,(tmin+logs.start_time,tmax+logs.start_time))
        
        fig.canvas.draw_idle()
        
    span_track = SpanSelector(tracking_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)
    span_speed = SpanSelector(speed_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)
    span_sep   = SpanSelector(sep_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)    
    
    return fig,traj_ax,[tracking_ax,speed_ax,sep_ax],[span_track,span_speed,span_sep]

def _plot_trajs(traj_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str], 
                hide_refs:bool, hide_reschedules:bool, hide_keyframes:bool,
                timewindow:Optional[tuple[float,float]] = None,
                alpha:float=1.0,
                labels:bool=True) -> list[tuple[Line2D,list[PathCollection],list[Quiver]]]:
    all_lines = []
    
    trajs = logs.as_trajectories()
    refs = logs.ref_trajectories()
    replanning_timestamps = logs.replanning_timestamps
    stat_list = logs.ac_stats
    
    if timewindow is not None:
        for k,v in trajs.items():
            trajs[k] = list(filter(lambda t : timewindow[0] <= t[0] <= timewindow[1],v))
        for k,v in refs.items():
            refs[k] = list(filter(lambda t : timewindow[0] <= t[0] <= timewindow[1],v))
        replanning_timestamps = list(filter(lambda t : timewindow[0] <= t <= timewindow[1],replanning_timestamps))
            
    
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
            
    
    _,l1s = plot_several_pose2d_sequences(traj_ax,trajs,color_dict,
        label=labels,alpha=alpha,endpoints=True)
    
    all_lines.extend(l1s.values())
    
    if not hide_refs:
        _,d = plot_several_pose2d_sequences(traj_ax,refs,color_dict,
            label=labels,alpha=alpha,endpoints=True,linestyle='dashed')
        for e in d.values():
            l = e[0].get_label()
            e[0].set_label(l + " (ref)")
        
        all_lines.extend(d.values())
    
    
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
                r_poses.append((t,r_interp(t)[0]))
                a_poses.append((t,a_interp(t)[0]))
            actual_replanning_poses[id] = a_poses
            ref_replanning_poses[id] = r_poses

        _,d1 =plot_several_pose2d_sequences(traj_ax,actual_replanning_poses,color_dict,
            label=labels,alpha=alpha,endpoints=False,linestyle=' ',marker="P",markersize=10)
        first = True
        for e in d1.values():
            if first:
                l = e[0].get_label()
                e[0].set_label(l + " (reschedule)")
                first = False
            else:
                e[0].set_label("")
                
        all_lines.extend(d1.values())
    
        if not hide_refs:
            _,d2 = plot_several_pose2d_sequences(traj_ax,ref_replanning_poses,color_dict,
                label=labels,alpha=alpha,endpoints=False,linestyle=' ',marker=(5,1,0),markersize=10)
            first = True
            for e in d2.values():
                if first:
                    l = e[0].get_label()
                    e[0].set_label(l + " (ref reschedule)")
                    first = False
                else:
                    e[0].set_label("")
            all_lines.extend(d2.values())
        
    traj_ax.set_xlabel("Easting (m)")
    traj_ax.set_ylabel("Northing (m)")
    
    if labels:
        traj_ax.legend()
    
    return all_lines

def _plot_separation(sep_ax:Axes, logs:TrackingLogs):
    separations = logs.get_all_min_sep()
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    
    ts = [t[0]-logs.start_time for t in separations]
    vals = [t[1] for t in separations]
    sep_ax.plot(ts,vals)
    sep_ax.set_ylim(0.,100)
    sep_ax.hlines(logs.separation_threshold,ts[0],ts[-1],colors='k',linestyles='dashed',label=f'Required Separation: {logs.separation_threshold:.1f} m')
    sep_ax.vlines(replanning_timestamps,0.,100,colors='grey',linestyles='dotted',label='Effective replanning')
    sep_ax.set_xlabel("Time since takeoff (s)")
    sep_ax.set_ylabel("Minimum Separation (m)")
    sep_ax.grid(True,'both','y')
    sep_ax.legend()
    
def _plot_speed(speed_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str]):
    
    speeds = logs.get_speeds()
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    
    min_t = np.inf
    max_t = -np.inf
    max_val = -np.inf
    
    for id,data in speeds.items():
        ts = [t[0]-logs.start_time for t in data]
        min_t = min(min_t,min(ts))
        max_t = max(max_t,max(ts))
        vals = [t[1] for t in data]
        expected = [t[2] for t in data]
        max_val = max(max_val,max(vals))
        speed_ax.plot(ts,vals,label=f"{id}",color=color_dict[id],linestyle='-')
        speed_ax.plot(ts,expected,label=f"{id} ref",color=color_dict[id],linestyle=':')
        
    for stat in logs.ac_stats:
        speed_ax.hlines(stat.airspeed,min_t,max_t,colors=color_dict[stat.id],linestyles='dashed',label=f"{stat.id} nominal speed: {stat.airspeed:.1f} m/s")
    
    speed_ax.set_ymargin(0.05)
    ymin,ymax = speed_ax.get_ylim()
    speed_ax.vlines(replanning_timestamps,ymin,ymax,colors='grey',linestyles='dotted',label='Effective replanning')
    # tracking_ax.set_xlabel("Time (s)")
    speed_ax.set_ylabel("Airspeed (m/s)")
    speed_ax.set_xticks(replanning_timestamps)
    speed_ax.grid(True,'both','y')
    # speed_ax.legend() 
    
def _plot_tracking(tracking_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str]):
    
    errs = logs.get_errors()
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    
    min_t = np.inf
    max_t = -np.inf
    max_val = -np.inf
    
    avg_err = 0
    for id,data in errs.items():
        ts = [t[0]-logs.start_time for t in data]
        min_t = min(min_t,min(ts))
        max_t = max(max_t,max(ts))
        vals = [t[1] for t in data]
        avg_err += np.mean(vals)
        max_val = max(max_val,max(vals))
        tracking_ax.plot(ts,vals,color=color_dict[id])#,label=f"{id}")
    
    avg_err /= len(errs)
    tracking_ax.hlines(logs.tracking_error_threshold,min_t,max_t,colors='k',linestyles='dashed',label=f'Tracking error threshold: {logs.tracking_error_threshold:.1f} m')
    tracking_ax.set_ymargin(0.05)
    ymin,ymax = tracking_ax.get_ylim()
    tracking_ax.vlines(replanning_timestamps,ymin,ymax,colors='grey',linestyles='dotted',label='Effective replanning')
    # tracking_ax.set_xlabel("Time (s)")
    tracking_ax.set_ylabel("Tracking error (m)")
    tracking_ax.set_title(f"Tracking error (avg: {avg_err:.1f} m)")
    tracking_ax.set_xticks(replanning_timestamps)
    tracking_ax.grid(True,'both','y')
    tracking_ax.legend()    
    
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyse tracking logs")
    parser.add_argument("logfile",type=pathlib.Path,help="Path to the log file to analyse")
    parser.add_argument('-G',"--obstacles",dest='obstacles',type=pathlib.Path,default=None,help="Path to the obstacles JSON file to plot")
    parser.add_argument("--no-refs",dest='no_refs',action='store_true',help="Hide the reference trajectories in the plot (useful when they are too cluttered)")
    parser.add_argument("--no-reschedules",dest='no_reschedules',action='store_true',help="Hide the rescheduling points in the plot")
    parser.add_argument("--no-keyframes",dest='no_keyframes',action='store_true',help="Hide the keyframe poses in the plot")
    args = parser.parse_args()
    
    with open(args.logfile,"rb") as f:
        logs:TrackingLogs = pickle.load(f)
        assert isinstance(logs,TrackingLogs), "The provided log file does not contain a TrackingLogs object"
    
    colorlist = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow', 'brown', 'pink']
    color_dict = {}
    for i,stat in enumerate(logs.ac_stats):
        color_dict[stat.id] = colorlist[i%len(colorlist)]
        
    fig,traj_ax,axes,selectors = plot_trackingdata(logs,color_dict,args.obstacles,
                                                       args.no_refs,
                                                       args.no_reschedules,
                                                       args.no_keyframes)
    
    fig.set_size_inches(16,9)
    fig.tight_layout()
    plt.show()