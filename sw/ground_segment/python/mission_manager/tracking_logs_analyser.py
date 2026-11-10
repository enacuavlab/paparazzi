#!/usr/bin/env python3

import pickle
import pathlib
import argparse
import json

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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
                      show_refs:bool=True,
                      show_reschedules:bool=True,
                      show_keyframes:bool=True,
                      show_critical:bool=True,
                      interactive:bool=True,
                      sharex:bool=True) -> tuple[Figure,Axes,list[Axes],list[SpanSelector]]:
    ## Plotting
    fig,axes = plt.subplots(3,2,
                            sharex=sharex,sharey=False,
                            width_ratios=[2,1],)
        
    # Big ax for trajectories
    gs = axes[0,0].get_gridspec()
    axes[0,0].remove()
    axes[1,0].remove()
    axes[2,0].remove()
    
    traj_ax = fig.add_subplot(gs[:,0])
    
    _plot_trajs(traj_ax, logs, color_dict, show_refs, show_reschedules, show_keyframes, show_critical, labels=False, alpha=0.1)
    
    cover_lines = _plot_trajs(traj_ax, logs, color_dict, show_refs, show_reschedules, show_keyframes, show_critical)
        
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
            if l[0] is not None:
                l[0].remove()
            for e in l[1]:
                e.remove()
            for e in l[2]:
                e.remove()
        cover_lines.clear()
        
        cover_lines = _plot_trajs(traj_ax, logs, color_dict, show_refs, show_reschedules, show_keyframes, show_critical,(tmin+logs.start_time,tmax+logs.start_time))
        
        fig.canvas.draw_idle()
        
    if interactive:
        span_track = SpanSelector(tracking_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)
        span_speed = SpanSelector(speed_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)
        span_sep   = SpanSelector(sep_ax,onselect,'horizontal',useblit=True,drag_from_anywhere=True)
        selectors = [span_track,span_speed,span_sep]
    else:
        selectors = []
    
    return fig,traj_ax,[tracking_ax,speed_ax,sep_ax],selectors

def animate_trackingdata(logs: TrackingLogs, color_dict: dict[int,str],
                         trail_duration: float = 10.,
                         show_refs: bool = True,
                         show_reschedules: bool = True,
                         show_keyframes: bool = True) -> tuple[Figure,Axes,list[Axes],FuncAnimation]:
    if trail_duration <= 0.:
        raise ValueError("trail_duration must be positive")

    fig, traj_ax, axes, selectors = plot_trackingdata(
        logs, color_dict, show_refs, show_reschedules, show_keyframes,
        interactive=False, sharex=False)
    
    critical_timepoints = logs.critical_log_timepoints()
    critical_times = set(t for _,t in critical_timepoints)

    all_times = [t[0] for trajectories in logs.as_trajectories().values()
                 for t in trajectories]
    if not all_times:
        raise ValueError("Tracking logs contain no trajectory samples")

    replanning_times = {time for time in logs.replanning_timestamps
                        if min(all_times) <= time <= max(all_times)}
    frame_times_set = set(all_times) | replanning_times
    frame_times = np.arange(min(frame_times_set), max(frame_times_set), 0.5)
    frames = []
    
    # Freeze for 2s, i.e. for 20 frames when running at 10 fps
    for frame_index, time in enumerate(frame_times):
        previous_time = frame_times[frame_index - 1] if frame_index else -np.inf
        replanning_frame = (
            any(previous_time < event_time <= time
                for event_time in replanning_times)
            or time == frame_times[-1]
        )
        separation_loss_frame = any(
            previous_time < event_time <= time
            for event_time in critical_times
        )
        frames.append((time, replanning_frame, separation_loss_frame))
        freeze_frame = replanning_frame or separation_loss_frame
        if freeze_frame:
            frames.extend((time, replanning_frame, separation_loss_frame) for _ in range(19))
            
    artists = (list(traj_ax.lines) + list(traj_ax.collections)
               + list(traj_ax.artists) + list(traj_ax.texts))

    def remove_artists(current_artists):
        for artist in current_artists:
            artist.remove()

    def update(frame):
        nonlocal artists
        time, replanning, separation_loss = frame
        remove_artists(artists)
        artists = []
        plotted = _plot_trajs_interpolated(
            traj_ax, logs, color_dict, show_refs, show_reschedules,
            True if replanning else show_keyframes, True,
            (time - trail_duration, time), labels=False,color_keyframes=True)
        for line, scatters, quivers in plotted:
            if line is not None:
                artists.append(line)
            artists.extend(scatters)
            artists.extend(quivers)
        title = f"Time since Fleet Manager start: {time - logs.start_time:.1f} s"
        if replanning:
            title = "(Replanning) " + title
        if separation_loss:
            title = "(Separation Loss) " + title
        traj_ax.set_title(title)
        timewindow = (time - trail_duration, time)
        tracking_ax, speed_ax, sep_ax = axes
        tracking_ax.clear()
        _plot_tracking(tracking_ax, logs, color_dict, timewindow)
        speed_ax.clear()
        _plot_speed(speed_ax, logs, color_dict, timewindow)
        sep_ax.clear()
        _plot_separation(sep_ax, logs, None, timewindow=timewindow)
        for axis in axes:
            axis.set_xlim(timewindow[0] - logs.start_time,
                          timewindow[1] - logs.start_time)
        return artists

    animation = FuncAnimation(
        fig, update, frames=frames, interval=1000 / 10,
        repeat=False, blit=False)
    return fig, traj_ax, axes, animation

def _plot_trajs_interpolated(traj_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str], 
                show_refs:bool, show_reschedules:bool, show_keyframes:bool, show_critical:bool,
                timewindow:Optional[tuple[float,float]],
                freq:float=1.0,
                alpha:float=1.0,
                labels:bool=True,
                color_keyframes:bool=False,
                ) -> list[tuple[Line2D,list[PathCollection],list[Quiver]]]:
    if freq <= 0.:
        raise ValueError("freq must be positive")

    logs.sort_logs()
    sample_period = 1. / freq
    all_times = [data.timestamp for log in logs.logs.values() for data in log]
    if not all_times:
        return []

    if timewindow is None:
        plot_start, plot_end = min(all_times), max(all_times)
    else:
        plot_start, plot_end = timewindow
        if plot_start > plot_end:
            raise ValueError("timewindow start must not be after its end")

    def sample_trajectories(interpolator_factory, require_reference=False):
        sampled = {}
        for ac_id, log in logs.logs.items():
            if require_reference and not any(data.expected_pose is not None for data in log):
                continue
            start = max(plot_start, log[0].timestamp)
            end = min(plot_end, log[-1].timestamp)
            if start > end:
                continue
            times = np.arange(start, end, sample_period)
            times = np.append(times, end)
            times = np.unique(times)
            interpolator = interpolator_factory(ac_id)
            sampled[ac_id] = [(float(timestamp), interpolator(timestamp)[0])
                              for timestamp in times]
        return sampled

    trajs = sample_trajectories(logs.traj_interpolator)
    refs = sample_trajectories(logs.ref_interpolator, require_reference=True)
    replanning_timestamps = [timestamp for timestamp in logs.replanning_timestamps
                             if plot_start <= timestamp <= plot_end]
    return _plot_trajs_data(
        traj_ax, logs, color_dict, trajs, refs, replanning_timestamps,
        show_refs, show_reschedules, show_keyframes, show_critical,
        plot_start, plot_end, alpha, labels, color_keyframes)

def _plot_trajs(traj_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str],
                show_refs:bool, show_reschedules:bool, show_keyframes:bool,
                show_critical:bool,
                timewindow:Optional[tuple[float,float]] = None,
                alpha:float=1.0,
                labels:bool=True,
                color_keyframes:bool=False,
                ) -> list[tuple[Line2D,list[PathCollection],list[Quiver]]]:
    trajs = logs.as_trajectories()
    refs = logs.ref_trajectories()
    replanning_timestamps = logs.replanning_timestamps
    plot_start = None
    plot_end = None

    if timewindow is not None:
        plot_start, plot_end = timewindow
        trajs = {
            ac_id: [sample for sample in values
                    if plot_start <= sample[0] <= plot_end]
            for ac_id, values in trajs.items()}
        refs = {
            ac_id: [sample for sample in values
                    if plot_start <= sample[0] <= plot_end]
            for ac_id, values in refs.items()}
        replanning_timestamps = [timestamp for timestamp in replanning_timestamps
                                 if plot_start <= timestamp <= plot_end]
        trajs = {ac_id: values for ac_id, values in trajs.items() if values}
        refs = {ac_id: values for ac_id, values in refs.items() if values}

    return _plot_trajs_data(
        traj_ax, logs, color_dict, trajs, refs, replanning_timestamps,
        show_refs, show_reschedules, show_keyframes, show_critical,
        plot_start, plot_end, alpha, labels, color_keyframes)

def _plot_trajs_data(traj_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str],
                     trajs:DictOfPoseTrajectories,
                     refs:DictOfPoseTrajectories,
                     replanning_timestamps:list[int|float],
                     show_refs:bool, show_reschedules:bool,
                     show_keyframes:bool, show_critical:bool,
                     plot_start:Optional[float] = None,
                     plot_end:Optional[float] = None,
                     alpha:float=1.0,
                     labels:bool=True,
                     color_keyframes:bool=False,
                     ) -> list[tuple[Line2D,list[PathCollection],list[Quiver]]]:
    all_lines = []
    stat_list = logs.ac_stats
    
    traj_ax.set_aspect('equal')
    
    if show_keyframes:
        keyframe_poses = logs.keyframes.keyposes
        for i,pose_set in enumerate(keyframe_poses):
            xs = pose_set[:,0]
            ys = pose_set[:,1]
            dxs = np.cos(pose_set[:,3])
            dys = np.sin(pose_set[:,3])
            keyframe_colors = 'k'
            if color_keyframes:
                keyframe_colors = [color_dict.get(stat.id, 'k')
                                   for stat in logs.ac_stats[:len(pose_set)]]
                keyframe_colors.extend(['k'] * (len(pose_set) - len(keyframe_colors)))
            keyframe_quiver = traj_ax.quiver(xs,ys,dxs,dys,color=keyframe_colors,
                                             angles='xy',)
            keyframe_label = traj_ax.text(float(np.mean(xs))+10, float(np.mean(ys)), str(i+1), color='k', fontsize=12, fontweight='bold',
                                          ha='center', va='center')
            all_lines.append((None, [keyframe_label], [keyframe_quiver]))

    if show_critical:
        critical_timepoints = logs.critical_log_timepoints()
        if plot_start is not None and plot_end is not None:
            critical_timepoints = [point for point in critical_timepoints
                                   if plot_start <= point[1] <= plot_end]
        for _, timestamp in critical_timepoints:
            _, first_id, second_id = logs.get_min_sep_at_timestamp(timestamp)
            poses = logs.get_poses_at_timestamp(timestamp)
            first_pose = poses[first_id]
            second_pose = poses[second_id]
            critical_line = traj_ax.plot(
                [first_pose.x, second_pose.x],
                [first_pose.y, second_pose.y],
                linestyle=':', marker='x', color='k',
                markersize=8, markeredgewidth=2)[0]
            all_lines.append((critical_line, [], []))
            
    
    _,l1s = plot_several_pose2d_sequences(traj_ax,trajs,color_dict,
        label=labels,alpha=alpha,endpoints=True)
    
    all_lines.extend(l1s.values())
    
    if show_refs:
        _,d = plot_several_pose2d_sequences(traj_ax,refs,color_dict,
            label=labels,alpha=alpha,endpoints=True,linestyle='dashed')
        for e in d.values():
            l = str(e[0].get_label())
            e[0].set_label(l + " (ref)")
        
        all_lines.extend(d.values())
    
    
    if show_reschedules:
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

        actual_replanning_poses = {k: v for k, v in actual_replanning_poses.items() if v}
        ref_replanning_poses = {k: v for k, v in ref_replanning_poses.items() if v}

        _,d1 =plot_several_pose2d_sequences(traj_ax,actual_replanning_poses,color_dict,
            label=labels,alpha=alpha,endpoints=False,linestyle=' ',marker="P",markersize=10)
        first = True
        for e in d1.values():
            if first:
                l = str(e[0].get_label())
                e[0].set_label(l + " (reschedule)")
                first = False
            else:
                e[0].set_label("")
                
        all_lines.extend(d1.values())
    
        if show_refs:
            _,d2 = plot_several_pose2d_sequences(traj_ax,ref_replanning_poses,color_dict,
                label=labels,alpha=alpha,endpoints=False,linestyle=' ',marker=(5,1,0),markersize=10)
            first = True
            for e in d2.values():
                if first:
                    l = str(e[0].get_label())
                    e[0].set_label(l + " (ref reschedule)")
                    first = False
                else:
                    e[0].set_label("")
            all_lines.extend(d2.values())
        
    traj_ax.set_xlabel("Easting (m)")
    traj_ax.set_ylabel("Northing (m)")
    
    if labels:
        traj_ax.legend(loc='upper left')
    
    return all_lines

def _plot_separation(sep_ax:Axes, logs:TrackingLogs, max_separation:Optional[float]=100.,
                     timewindow:Optional[tuple[float,float]] = None):
    separations = logs.get_all_min_sep()
    if timewindow is not None:
        separations = [sample for sample in separations
                       if timewindow[0] <= sample[0] <= timewindow[1]]
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    if timewindow is not None:
        replanning_timestamps = [t - logs.start_time
                                 for t in logs.replanning_timestamps
                                 if timewindow[0] <= t <= timewindow[1]]
    
    ts = [t[0]-logs.start_time for t in separations]
    vals = [t[1] for t in separations]
    if ts:
        sep_ax.plot(ts,vals)
    if max_separation is not None:
        sep_ax.set_ylim(0.,max_separation)
    if ts:
        sep_ax.hlines(logs.separation_threshold,ts[0],ts[-1],colors='k',linestyles='dashed',label=f'Required Separation: {logs.separation_threshold:.1f} m')
    
    ylims = sep_ax.get_ylim()
    sep_ax.vlines(replanning_timestamps,ylims[0],ylims[1],colors='grey',linestyles='dotted',label='Effective replanning')
    sep_ax.set_xlabel("Time since Fleet Manager start (s)")
    sep_ax.set_ylabel("Minimum Separation (m)")
    sep_ax.grid(True,'both','y')
    sep_ax.legend()
    
def _plot_speed(speed_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str],
                timewindow:Optional[tuple[float,float]] = None):
    
    speeds = logs.get_speeds()
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    if timewindow is not None:
        replanning_timestamps = [t - logs.start_time
                                 for t in logs.replanning_timestamps
                                 if timewindow[0] <= t <= timewindow[1]]
    
    min_t = np.inf
    max_t = -np.inf
    max_val = -np.inf
    
    for id,data in speeds.items():
        if timewindow is not None:
            data = [sample for sample in data
                    if timewindow[0] <= sample[0] <= timewindow[1]]
        if not data:
            continue
        ts = [t[0]-logs.start_time for t in data]
        min_t = min(min_t,min(ts))
        max_t = max(max_t,max(ts))
        vals = [t[1] for t in data]
        expected = [t[2] for t in data]
        max_val = max(max_val,max(vals))
        speed_ax.plot(ts,vals,label=f"{id}",color=color_dict[id],linestyle='-')
        speed_ax.plot(ts,expected,label=f"{id} ref",color=color_dict[id],linestyle=':')
        
    if min_t <= max_t:
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
    
def _plot_tracking(tracking_ax:Axes, logs:TrackingLogs, color_dict:dict[int,str],
                   timewindow:Optional[tuple[float,float]] = None):
    
    errs = logs.get_errors()
    replanning_timestamps = [t - logs.start_time for t in logs.replanning_timestamps]
    if timewindow is not None:
        replanning_timestamps = [t - logs.start_time
                                 for t in logs.replanning_timestamps
                                 if timewindow[0] <= t <= timewindow[1]]
    
    min_t = np.inf
    max_t = -np.inf
    max_val = -np.inf
    
    avg_err = 0
    valid_means = []
    for id,data in errs.items():
        if timewindow is not None:
            data = [sample for sample in data
                    if timewindow[0] <= sample[0] <= timewindow[1]]
        ts = np.array([t[0]-logs.start_time for t in data])
        vals = np.array([t[1] for t in data])
        valids = np.isnan(vals) == False
        ts = ts[valids]
        vals = vals[valids]
        if not len(vals):
            continue
        min_t = min(min_t,min(ts))
        max_t = max(max_t,max(ts))
        mean_err = np.mean(vals)
        avg_err += mean_err
        valid_means.append(mean_err)
        max_val = max(max_val,max(vals))
        if timewindow is None:
            print(f"Plotting tracking error for {id}: {len(ts)} points, mean: {np.mean(vals):.2f} m, max: {np.max(vals):.2f} m")
        tracking_ax.plot(ts,vals,color=color_dict[id])#,label=f"{id}")
    
    if valid_means:
        avg_err /= len(valid_means)
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
    parser.add_argument("--no-critical",dest='no_critical',action='store_true',help="Hide the critical separation points in the plot (ignored if --animate is used)")
    parser.add_argument("-a", "--animate", nargs="?", type=float, const=10., default=None,
                        metavar="SECONDS", help="Animate trajectories with this trail duration (default: 10 s)")
    parser.add_argument("--map", type=str, default=None,
                        help="Projection of log coordinates; add satellite imagery")
    parser.add_argument("--local",dest='local',action='store_true',help="Zero the coordinates to the local reference frame (min x,y = 0)")
    parser.add_argument("--save",dest='save',type=pathlib.Path,default=None,help="Path to save the figure (if not provided, the figure will be shown instead)")
    args = parser.parse_args()
    
    with open(args.logfile,"rb") as f:
        logs:TrackingLogs = pickle.load(f)
        assert isinstance(logs,TrackingLogs), "The provided log file does not contain a TrackingLogs object"
    
    colorlist = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow', 'brown', 'pink']
    color_dict = {}
    for i,stat in enumerate(logs.ac_stats):
        color_dict[stat.id] = colorlist[i%len(colorlist)]
    
    if args.local:
        shift_x,shift_y,shift_t = logs.get_local_reference()
        logs.start_time = shift_t
    
    if args.animate is None:
        fig,traj_ax,axes,selectors = plot_trackingdata(logs,color_dict,
                                                           not args.no_refs,
                                                           not args.no_reschedules,
                                                           not args.no_keyframes,
                                                           not args.no_critical)
    else:
        args.no_keyframes = True  # Hide keyframes in animation for clarity
        fig,traj_ax,axes,animation = animate_trackingdata(
            logs, color_dict, args.animate,
            not args.no_refs,
            not args.no_reschedules,
            not args.no_keyframes)
    
    if args.obstacles is not None:
        with open(args.obstacles) as f:
            for e in json.load(f)["sections"]:
                s = parse_section_from_dict(e)
                plot_BasicPath_obstacle(traj_ax,s)
    
    if args.local:
        shift_x,shift_y,shift_t = logs.get_local_reference()
        
        # Positions
        local_xrange = (traj_ax.get_xlim()[0]-shift_x,traj_ax.get_xlim()[1]-shift_x)
        local_yrange = (traj_ax.get_ylim()[0]-shift_y,traj_ax.get_ylim()[1]-shift_y)
        
        local_xmin = np.ceil(local_xrange[0]/100.)*100.
        local_xmax = np.floor(local_xrange[1]/100.)*100.
        local_ymin = np.ceil(local_yrange[0]/100.)*100.
        local_ymax = np.floor(local_yrange[1]/100.)*100.
        
        new_xticks_labels = np.arange(local_xmin,local_xmax+1,100.)
        new_xticks = new_xticks_labels + shift_x
        new_yticks_labels = np.arange(local_ymin,local_ymax+1,100.)
        new_yticks = new_yticks_labels + shift_y
        
        traj_ax.set_xticks(new_xticks)
        traj_ax.set_xticklabels([f"{x:.0f}" for x in new_xticks_labels])
        traj_ax.set_yticks(new_yticks)
        traj_ax.set_yticklabels([f"{y:.0f}" for y in new_yticks_labels])
        
        traj_ax.set_xlabel(f"Local Easting (m)")
        traj_ax.set_ylabel(f"Local Northing (m)")
        
        # Time
        tticks = axes[-1].get_xticks()
        new_tticklabels = tticks - shift_t + logs.start_time
        
        axes[-1].set_xticklabels([f"{x:.1f}" for x in new_tticklabels])

        axes[-1].set_xlabel(f"Time since Fleet Manager start (s)")

    if args.map is not None:
        try:
            import contextily as ctx
        except ImportError as error:
            raise RuntimeError(
                "Satellite imagery requires the 'contextily' package"
            ) from error
        ctx.add_basemap(
            traj_ax, source="Esri.WorldImagery",
            crs=args.map, reset_extent=True,alpha=0.5
        )
    
    fig.set_size_inches(16,9)
    fig.tight_layout()
    if args.save is not None:
        if args.animate is not None:
            def show_progress(current_frame: int, total_frames: int) -> None:
                progress = (current_frame + 1) / total_frames
                bar_width = 40
                completed = int(bar_width * progress)
                bar = '#' * completed + '-' * (bar_width - completed)
                print(f'\rSaving animation: [{bar}] {progress:.0%}', end='', flush=True)

            animation.save(
                args.save, writer='ffmpeg', fps=10,extra_args=['-preset', 'ultrafast','-vcodec', 'libx264'],
                progress_callback=show_progress)
            print()
        else:
            fig.savefig(args.save)
    else:
        plt.show()