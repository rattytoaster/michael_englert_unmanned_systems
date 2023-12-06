#!/usr/bin/env python3

import rclpy
import math as m
import numpy as np
import threading 


from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from unmanned_systems_ros2_pkg import ProNav

from pathfinding_lib import Djikstras

import matplotlib.pyplot as plt


def get_mean_heading_target(heading_list:list)-> float:
     heading_list = np.array(heading_list)
     mean_heading_target = np.mean(heading_list)
     return mean_heading_target   

def compute_global_heading(heading_target_rad:float, 
                        curent_yaw_rad:float):
    
    global_heading_rad = heading_target_rad + curent_yaw_rad
    
    if global_heading_rad > 2*np.pi:
        global_heading_rad = global_heading_rad - 2*np.pi
    elif global_heading_rad < 0:
        global_heading_rad = global_heading_rad + 2*np.pi
    
    # print("global heading deg", np.rad2deg(global_heading_rad))
    
    return global_heading_rad

def main() -> None:
    rclpy.init(args=None)

    #lidar frequency 
    lidar_freq = 5.0 #hz

    turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')    
    turtlebot_pursuer.move_turtle(0.0,0.0)
    
    #since our lidar is super slow we're going to set this node to match our
    #lidar rate to about 3 times its sampling rate
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_pursuer, ), 
                              daemon=True)
    thread.start()
    rate = turtlebot_pursuer.create_rate(lidar_freq*3)
    
    # 5 works well for my side increase to make it more snappier on turns 
    
    #this value works well with simple pn
    #pro_nav = ProNav.ProNav(1.5)    
    
    #this value works well with true pn
    pro_nav = ProNav.ProNav(2.0)

    #This value works well with augmented pn
    #pro_nav = ProNav.ProNav(3.0)    
    
    dt = 1/lidar_freq
    old_evader_position = np.array([2,1])

    obstacles = [(5,0), (5,1), (5,2), (5,3), (5,4), (0,5), (1,4), (2,3), (3,2), (3,3)]

    min_x = -1.0
    max_x = 10.0
    min_y = -1.0
    max_y = 10.0
    radius = 0.5
    gs = 0.5
    start_x = 2.0
    start_y = 1.0

    goal = (7.0,2.0)
    #init_straight_path = Djikstra_Astar.astar(-1.0,10.0,-1.0,10.0,obstacles,radius,pos_curr[0],pos_curr[1],goal_x,goal_y,gs,radius)
    init_straight_path = Djikstras.Djikstras.djikstras(min_x,max_x,min_y,max_y,obstacles,radius,start_x,start_y,goal[0],goal[1],gs,0.0)
    
    #print("path:", init_straight_path)

    fig, ax = plt.subplots()

    ax.plot(init_straight_path[0],init_straight_path[1], 'r')
    for obs in obstacles:
        obs_plot = plt.Circle((obs[0], obs[1]), radius, color='red')
        ax.add_patch(obs_plot)



    init_x_path = init_straight_path[0]
    init_y_path = init_straight_path[1]
    straight_path = []
    path_len = len(init_straight_path[0])
    for i in range(path_len):
            
        straight_path.append((init_x_path[(path_len-1)-i],init_y_path[(path_len-1)-i]))

    print('path:', straight_path)

    waypoints = straight_path
    actual_path_x = []
    actual_path_y = []

    pursuer_path_x = []
    pursuer_path_y = []
    
    while rclpy.ok():
        
        # rclpy.spin_once(turtlebot_pursuer)
        actual_path_x.append(turtlebot_pursuer.evader_position[0])
        actual_path_y.append(turtlebot_pursuer.evader_position[1])

        pursuer_path_x.append(turtlebot_pursuer.current_position[0])
        pursuer_path_y.append(turtlebot_pursuer.current_position[1])

        closing_dist = np.sqrt((turtlebot_pursuer.current_position[0]-turtlebot_pursuer.evader_position[0])**2 + (turtlebot_pursuer.current_position[0]-turtlebot_pursuer.evader_position[0])**2)
        print("closing dist:", closing_dist)
        if closing_dist < 0.25:
            del(actual_path_x[0])
            del(actual_path_y[0])

            ax.plot(actual_path_x, actual_path_y, 'b')
            ax.plot(pursuer_path_x, pursuer_path_y, 'y')
            plt.show()

        rate.sleep()
        
        mean_target = get_mean_heading_target(
            turtlebot_pursuer.detected_heading_angle_list)
                 
        global_heading_ref = compute_global_heading(
            np.deg2rad(mean_target), turtlebot_pursuer.orientation_euler[2]
        )
            
        evader_position = np.array(turtlebot_pursuer.evader_position)

        evader_velocity = (evader_position - old_evader_position)/dt
                
        # flight_path_rate, cmd_vel = pro_nav.true_pro_nav(
        #     np.array(turtlebot_pursuer.current_position), 
        #     evader_position,
        #     dt, 
        #     evader_velocity, 
        #     np.array(turtlebot_pursuer.current_velocity),
        #     True, global_heading_ref    
        # )
        
        #cmd_vel = 0.20
        #flight_path_rate = pro_nav.simple_pro_nav(
        #    global_heading_ref, dt
        #)

        pursuer_pos_global_x = turtlebot_pursuer.current_position[0]
        pursuer_pos_global_y = turtlebot_pursuer.current_position[1]

        pursuer_vel_x = turtlebot_pursuer.current_velocity[0]        
        pursuer_vel_y = turtlebot_pursuer.current_velocity[1]

        pursuer_pos = np.array([pursuer_pos_global_x,pursuer_pos_global_y])
        pursuer_vel = np.array([pursuer_vel_x,pursuer_vel_y])

        #for true pro nav
        flight_path_rate, cmd_vel = pro_nav.true_pro_nav(pursuer_pos,evader_position,dt,evader_velocity,pursuer_vel,True,global_heading_ref)
        
        #for simple pro nav
        #flight_path_rate = pro_nav.simple_pro_nav(global_heading_ref,dt)    
        #cmd_vel = 0.2

        # do this command for half a second        
        print("flight path rate", flight_path_rate)
        old_evader_position = evader_position
        turtlebot_pursuer.move_turtle(cmd_vel, flight_path_rate)
        
    
if __name__ == '__main__':
    main()
    


