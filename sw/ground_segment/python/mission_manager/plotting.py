# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

#!/usr/bin/env python3

import numpy as np
import typing
from typing import Optional
import pathlib

from _plotting_extra import linestyle_dict,my_cmap,ColorType
from Dubins import FleetPlan,Pose3D,ListOfTimedPoses,DictOfPoseTrajectories,min_XY_dist
from ioUtils import parse_trajectories_from_JSON,parse_trajectories_from_CSV,transpose_list_of_trajectories



#################### Plotting ####################

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
from matplotlib.text import Text
from matplotlib import transforms,colormaps


ARROWS_SCALING = {
    'scale_units':'inches',
    'scale' : 4,
    'units' : 'inches',
    'width' : 0.03
}

def plot_pose2d_sequence(ax:Axes,poses:list[Pose3D],endpoints:bool=False,endarrows:bool=False,**plot_kwargs) -> tuple[Axes,list[Line2D]]:
    xs = np.asarray([p.x for p in poses])
    ys = np.asarray([p.y for p in poses])
    angles = np.asarray([p.theta for p in poses])
    
    lines = ax.plot(xs,ys,**plot_kwargs)
    
    # Avoid repeating the name in the legend
    plot_kwargs.pop('label',None)
    plot_kwargs.pop('angles',None)
    plot_kwargs.pop('marker',None)
    
    try:
        color = plot_kwargs['color']
    except KeyError:
        color = None
    
    if (endpoints):
        ax.scatter(xs[0],ys[0],marker='o',color=color)
        ax.scatter(xs[-1],ys[-1],marker='X',color=color)
    
    
    if (endarrows):
        ax.quiver([xs[0]],[ys[0]],[np.cos(angles[0])],[np.sin(angles[0])],angles='xy',color='k')
        ax.quiver([xs[-1]],[ys[-1]],[np.cos(angles[-1])],[np.sin(angles[-1])],angles='xy',color='k')
    
    return ax,lines

def plot_several_pose2d_sequences(ax:Axes,poses_dict:DictOfPoseTrajectories,colors_dict:typing.Optional[dict[int,ColorType]]=None,
                                  endpoints:bool=False,endarrows:bool=False,label:bool=False,alpha:float=1.,**plot_kwargs) -> tuple[Axes,dict[int,Line2D]]:
    i = 0
    
    output = dict()
    for key,values in poses_dict.items():
        pose_seq = [v[1] for v in values]
        c = colors_dict[key] if colors_dict is not None else my_cmap[i % len(my_cmap)]
        if label:
            ax,l = plot_pose2d_sequence(ax,pose_seq,endpoints,endarrows,
                                    color=c,
                                    label=f"AC: {key}",
                                    alpha=alpha,
                                    **plot_kwargs)
        else:
            ax,l = plot_pose2d_sequence(ax,pose_seq,endpoints,endarrows,
                                    color=c,
                                    alpha=alpha,
                                    **plot_kwargs)
            
        output[key] = l[0]
        
        i += 1
        
        
    return ax,output


def snapshot_several_pose2d_sequences(poses_list:ListOfTimedPoses,snapshots:int,
                                     color_dict:typing.Optional[dict[int,ColorType]]=None,
                                     name_dict:typing.Optional[dict[int,str]]=None,
                                     save_fig:typing.Optional[str]=None,
                                     show_fig:bool=False,
                                     no_legend:bool=False,
                                     xlim:Optional[tuple[float,float]] = None):
    # Set axes aspect
    fig,ax = plt.subplots(figsize=(16,9))
    if xlim is None:
        ax.set_aspect('equal','datalim')
    else:
        ax.set_aspect('equal')
    
    # Sort in ascending time
    poses_list.sort(key=lambda t : t[0])
    
    poses_dict = transpose_list_of_trajectories(poses_list)
    
    # Setup color scheme
    if color_dict is None:
        color_dict = dict()
        i = 0
        for k in poses_dict.keys():
            color_dict[k] = my_cmap[i % len(my_cmap)]
            i += 1
    
    # Plot background lines
    ax,_ = plot_several_pose2d_sequences(ax,poses_dict,color_dict,False,False,False,0.3)
    
    # Plot min dist location
    mdist = np.inf
    time_mdist = 0.
    p1_mdist = Pose3D.undefined()
    p2_mdist = Pose3D.undefined()
    i1_mdist = 0
    i2_mdist = 0
    for time,d in poses_list:
        now_names = []
        now_poses = []
        for k,v in d.items():
            now_names.append(k)
            now_poses.append(v)
            
        local_mdist,i1,i2 = min_XY_dist(now_poses)
        if local_mdist < mdist:
            mdist = local_mdist
            time_mdist = time
            p1_mdist = now_poses[i1]
            p2_mdist = now_poses[i2]
            i1_mdist = now_names[i1]
            i2_mdist = now_names[i2]
    
    
    arrow_indexes = np.round(np.linspace(0,len(poses_list)-1,snapshots,endpoint=True)).astype(int)
    
    keys = list(poses_list[0][1].keys())
    keys.sort()
    for i in arrow_indexes:
        poses = poses_list[i][1].values()
        xs = [p.x for p in poses]
        ys = [p.y for p in poses]
        thetas = [p.theta for p in poses]
        colors = [color_dict[t] for t in keys]
        
        ax.quiver(
            xs,ys,np.cos(thetas),np.sin(thetas),
            color=colors,
            angles='xy',**ARROWS_SCALING
        )
        
        ax.scatter(
            xs,ys,c=colors,marker='+',
        )
        
        
    
    # Setup artists for animation
    
    init_poses = [poses_list[0][1][k] for k in keys]
    
    for id,pose in zip(keys,init_poses):
        label = f"AC: {id}"
        if name_dict is not None:
            label += " " + name_dict[id]
            
        ax.scatter([pose.x],[pose.y],color=color_dict[id],
                   marker='o',label=label,alpha=0.5)
    for id,pose in poses_list[-1][1].items():
        ax.scatter([pose.x],[pose.y],color=color_dict[id],
                   marker='X',alpha=0.5)
        

    
    agent_num_txtwidth = int(np.log10(len(keys)))+1
    
    if not(no_legend):
        mindist_artist = ax.plot(
            [p1_mdist.x,p2_mdist.x],
            [p1_mdist.y,p2_mdist.y],
            marker='D',color='k',linestyle=':',markeredgecolor='r',
            label=f"Min dist at time {time_mdist:.1f} ({i1_mdist:{agent_num_txtwidth}},{i2_mdist:{agent_num_txtwidth}}): {mdist:.3f}"
        )[0]
        
    else:
        mindist_artist = ax.plot(
            [p1_mdist.x,p2_mdist.x],
            [p1_mdist.y,p2_mdist.y],
            alpha=0,
        )[0]
    
    if xlim is not None:
        ax.set_xlim(xlim)
    
    ax.set_title(f"Total flight time: {poses_list[-1][0]:.2f}")
    ax.set_xmargin(0.2)
    ax.set_ymargin(0.2)
    legend = ax.legend()
    if no_legend:
        legend.remove()
        ax.set_axis_off()

    
    fig.tight_layout()
    if show_fig:
        plt.show()

def animate_several_pose2d_sequences(poses_list:ListOfTimedPoses,fps:int=30,
                                     color_dict:typing.Optional[dict[int,ColorType]]=None,
                                     name_dict:typing.Optional[dict[int,str]]=None,
                                     save_animation:typing.Optional[str]=None,
                                     show_animation:bool=False,
                                     no_legend:bool=False):
    # Set axes aspect
    fig,ax = plt.subplots(figsize=(16,9))
    ax.set_aspect('equal','datalim')
    
    # Sort in ascending time
    poses_list.sort(key=lambda t : t[0])
    
    poses_dict = transpose_list_of_trajectories(poses_list)
    
    # Setup color scheme
    if color_dict is None:
        color_dict = dict()
        i = 0
        for k in poses_dict.keys():
            color_dict[k] = my_cmap[i % len(my_cmap)]
            i += 1
    
    # Plot background lines
    ax,_ = plot_several_pose2d_sequences(ax,poses_dict,color_dict,False,False,False,0.2)
    
    # Setup artists for animation
    keys = list(poses_list[0][1].keys())
    keys.sort()
    
    init_poses = [poses_list[0][1][k] for k in keys]
    init_xs = [p.x for p in init_poses]
    init_ys = [p.y for p in init_poses]
    init_thetas = [p.theta for p in init_poses]
    init_colors = [color_dict[t] for t in keys]
    
    for id,pose in zip(keys,init_poses):
        label = f"AC: {id}"
        if name_dict is not None:
            label += " " + name_dict[id]
            
        ax.scatter([pose.x],[pose.y],color=color_dict[id],
                   marker='o',label=label,alpha=0.5)
    for id,pose in poses_list[-1][1].items():
        ax.scatter([pose.x],[pose.y],color=color_dict[id],
                   marker='X',alpha=0.5)
        
    
    pos_artist = ax.scatter(
        init_xs,init_ys,
        color=init_colors,label="Aircraft positions",marker='+'
    )
    
    dirs_artist = ax.quiver(
        init_xs,init_ys,np.cos(init_thetas),np.sin(init_thetas),
        color=init_colors,label="Aircraft direction",
        angles='xy',**ARROWS_SCALING
    )
    
    min_dist,i1,i2 = min_XY_dist(init_poses)
    id1,p1 = keys[i1],poses_list[0][1][keys[i1]]
    id2,p2 = keys[i2],poses_list[0][1][keys[i2]]
    
    agent_num_txtwidth = int(np.log10(len(init_poses)))+1
    
    if not(no_legend):
        mindist_artist = ax.plot(
            [p1.x,p2.x],
            [p1.y,p2.y],
            marker='D',color='k',linestyle=':',markeredgecolor='r',
            label=f"Min dist ({id1:{agent_num_txtwidth}},{id2:{agent_num_txtwidth}}): {min_dist:.3f}"
        )[0]
        
    else:
        mindist_artist = ax.plot(
            [p1.x,p2.x],
            [p1.y,p2.y],
            alpha=0,
        )[0]
    

    # Animation callback function
    def update(frame:int):
        modified_artists = []
        
        poses_and_ids = poses_list[frame][1]
        poses = [poses_and_ids[k] for k in keys]
        
        offsets = np.asarray([[p.x,p.y] for p in poses])
        angles = np.asarray([p.theta for p in poses])

        
        
        ### Positions
        
        
        min_dist,i1,i2 = min_XY_dist(poses)
        id1,p1 = keys[i1],poses[i1]
        id2,p2 = keys[i2],poses[i2]
        
        # Draw only if distance is relevant
        if not(no_legend):
            if min_dist < 1e4:
                mindist_artist.set_label(f"Min dist ({id1:{agent_num_txtwidth}},{id2:{agent_num_txtwidth}}): {min_dist:.3f}")
                mindist_artist.set_data([
                    [p1.x,p2.x],
                    [p1.y,p2.y],
                    ])
            else:
                mindist_artist.set_label(f"Min dist ----")
                mindist_artist.set_data([
                    [1e9,1e9],
                    [1e9,1e9]
                    ])
                
            modified_artists.append(mindist_artist)
        
        pos_artist.set_offsets(offsets)
        modified_artists.append(pos_artist)
        
        dirs = np.vstack([np.cos(angles),np.sin(angles)])
        
        dirs_artist.set_offsets(offsets)
        dirs_artist.set_UVC(dirs[0],dirs[1])
        modified_artists.append(dirs_artist)
        
        # wind_artist.set_offsets(poses[:,:2]+dirs.T)
        # modified_artists.append(wind_artist)
        
        # speed_artist.set_offsets(poses[:,:2])
        # speed_artist.set_UVC(dirs[0]+repeated_wind[:,0],dirs[1]+repeated_wind[:,1])
        # modified_artists.append(speed_artist)
        
        if not(no_legend):
            modified_artists.append(ax.legend())
            ax.set_title(f"Time: {poses_list[frame][0]:.4f}")
        
        return modified_artists
    
    fig.tight_layout()
    ax.set_xmargin(0.2)
    ax.set_ymargin(0.2)
    legend = ax.legend()
    if no_legend:
        legend.remove()
        ax.set_axis_off()
    
    ani = FuncAnimation(fig,update,repeat=True,frames=len(poses_list),interval=1000/fps)
    
    if save_animation is not None:
        print(f"Saving file to '{save_animation}' ...",end='',flush=True)
        if ".gif" in save_animation or ".webp" in save_animation or ".apng" in save_animation:
            ani.save(save_animation,fps=fps,writer="pillow")
        elif ".webm" in save_animation:
            ani.save(save_animation,fps=fps,writer="imagemagick",extra_args=["-quality", "100"])
        else:
            ani.save(save_animation,fps=fps)
        print(" Done!")
    
    if show_animation:
        plt.show()

def animate_fleet_plan(plan:FleetPlan,sample_num:int,fps:int=30,
                       color_dict:typing.Optional[dict[int,ColorType]]=None,
                       name_dict:typing.Optional[dict[int,str]]=None,
                       save_animation:typing.Optional[str]=None,
                       show_animation:bool=False,
                       no_legend:bool=False):
    
    samples = plan.sample_poses(sample_num)
    animate_several_pose2d_sequences(samples,
                                     fps,
                                     color_dict,
                                     name_dict,
                                     save_animation,
                                     show_animation,
                                     no_legend)



if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser('Dubins plotter','Display a Dubins path planning result.')
    parser.add_argument('file',type=str,help='File from which to parse data')
    parser.add_argument('save',type=str,nargs='?',default=None,help='Optional argument: if set, specifies where to save the animation')
    parser.add_argument('-np','--no-print',dest='no_print',action='store_true',help='Hide resulting animation if set')
    parser.add_argument('--fps',type=int,help="Number of Frames Per Second when building animation. Default is 30",default=30)
    parser.add_argument('-c','--colormap',type=str,help="A Matplotlib colormap name, mapping aircraft ID to color. Default to a qualitative map",default=None)
    parser.add_argument('--no-legend',dest='no_legend',action='store_true',help="Remove the legend for a cleaner plot. Default to False", default=False)
    parser.add_argument('-n','--sample-num',dest='sample_num',type=int,help="Number of samples to take to generate figure. Only relevant on JSON files. Default to 500",default=500)
    parser.add_argument('-s','--static',dest='static',type=int,
                        help="Print a static image with superposed snapshots instead of an animation. The parameter sets the number of snapshots. Must be a positive integer",
                        default=0)
    parser.add_argument('--xlim',nargs=2,dest='xlim',help="If defined, set the X-axis limits from plotting",default=None)
    
    args = parser.parse_args()
    
    path = pathlib.Path(args.file)
    save = args.save
    show_anim = not(args.no_print)
    sample_num = args.sample_num
    if not(path.is_file()):
        print(f"{path} is not a file!! Exiting....")
        exit(1)
        
    data:ListOfTimedPoses = []
    ext = path.suffix
    
    
    if ext.lower() == '.csv':
        print("Parsing...",end='')
        data = parse_trajectories_from_CSV(path)
        ids = [t[0] for t in data[0][1]]
        name_dict = None
        print(" Done!\nCreating animation...")
    elif ext.lower() == '.json':
        print("Parsing...",end='')
        plan = parse_trajectories_from_JSON(path)
        name_dict = plan.generate_id_name_dict()
        ids = name_dict.keys()
        data = plan.sample_poses(sample_num)
        
        print(" Done!\nCreating animation...")
    else:
        print(f"File extension not handled: {ext}\nExiting...")
        exit(1)
        
    
    min_id = min(ids)
    max_id = max(ids)
    
    if args.colormap is None:
        color_dict = None
    else:
        cmap = colormaps[args.colormap]
        color_dict = dict()
        for id in ids:
            color_dict[id] = cmap((id-min_id)/(max_id-min_id))
    
    
    if args.static > 0:
        snapshot_several_pose2d_sequences(data,args.static,color_dict,name_dict,save,show_anim,args.no_legend)
    else:
        animate_several_pose2d_sequences(data,args.fps,color_dict,name_dict,save,show_anim,args.no_legend)
    print("Done! Exiting...")