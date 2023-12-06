#!/usr/bin/env python3
import rclpy
import math as m

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools

import matplotlib.pyplot as plt

from pathfinding_lib import Djikstra_Astar
from pathfinding_lib import Node

from pathfinding_lib import Djikstras

import numpy as np

def generate_random_waypoints(n_random_waypoints:int, max_val:int)->list:
    """generate random waypoints from 1 to 1"""
    
    random_wp_list = []
    for i in range(0,n_random_waypoints+1):
        rand_x = randint(0, max_val)
        rand_y = randint(0, max_val)
        random_wp_list.append((rand_x, rand_y))
        
    return random_wp_list

def compute_desired_heading(current_pos:list, des_pos:list) -> float:
    """compute desired heading based on positions"""
    return m.atan2(des_pos[1] - current_pos[1] , des_pos[0] - current_pos[0])

def compute_dist_error(current_pos:list, des_pos:list)->float:
    """compute distance error"""
    return m.dist(des_pos,current_pos)

def compute_heading_error(current_heading:float, des_heading:float) -> float:
    """compute heading error in radians"""
    return des_heading - current_heading

def gimme_da_loot(turtlebot:TurtleBotNode, waypoint:list) -> list:
    """helper function"""
    desired_heading = compute_desired_heading(
        turtlebot.current_position, waypoint)
    
    heading_error = compute_heading_error(
        turtlebot.orientation_euler[2], desired_heading)

    dist_error = compute_dist_error(
        turtlebot.current_position, waypoint)
    
    return [desired_heading, heading_error, dist_error]
def get_mean_dist(dist_list:list)-> float:
    dist_list = np.array(dist_list)
    mean_dist = np.mean(dist_list)
    return mean_dist


def main() -> None:
    rclpy.init(args=None)
    
    turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'evader')    
    turtlebot_evader.move_turtle(0.0,0.0)

    #turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')   

    set_random = False
    is_done = False
    n_random_waypoints = 1
    heading_tol = 0.1; #radians
    dist_tolerance = 0.25 #meters

    turn_tolerance = 2.0
    
    turn_speed = 0.1 #rad/speed
    line_speed = 0.15 #m/s
    stop_speed = 0.0 

    closing_tol = 1.5
    
    if set_random == False:
        waypoints = [[7.0,2.0]]
        #waypoints = [[2.0,3.0]]
    else:
        waypoints = generate_random_waypoints(n_random_waypoints, 15);
    obstacles = [(5,0), (5,1), (5,2), (5,3), (5,4), (0,5), (1,4), (2,3), (3,2), (3,3)]
    #obstacles = []



    goal = waypoints[0]

    #print("goal:", goal)
    #print("pos curr:", turtlebot_evader.current_position[0], turtlebot_evader.current_position[1])
    min_x = -1.0
    max_x = 10.0
    min_y = -1.0
    max_y = 10.0
    radius = 0.5
    gs = 0.5
    start_x = 2.0
    start_y = 1.0
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

        if is_done == True:
            print("I'm done")
            turtlebot_evader.move_turtle(stop_speed, stop_speed)

            plt.show()
            rclpy.shutdown()

        for waypoint in waypoints:
            print("current waypoint is", waypoint)
            #if is_done == True:
                #break
            
            desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

            while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):
                #if is_done == True:
                    #break

                #print("pursuer pos:", turtlebot_pursuer.current_position)


                los_dist = get_mean_dist(turtlebot_evader.detected_range_list)
                #los_dist = np.sqrt((turtlebot_evader.current_position[0]-turtlebot_pursuer.current_position[0])**2 + (turtlebot_evader.current_position[1]-turtlebot_pursuer.current_position[1])**2)
                #print("closing dist:", los_dist)
                if los_dist < closing_tol:
                    is_done = True
                    #print("got yah!")

                #print("pos curr", turtlebot_evader.current_position[0], turtlebot_evader.current_position[1])
                #print("wp curr", waypoint)

                actual_path_x.append(turtlebot_evader.current_position[0])
                actual_path_y.append(turtlebot_evader.current_position[1])

                #pursuer_path_x.append(turtlebot_pursuer.current_position[0])
                #pursuer_path_y.append(turtlebot_pursuer.current_position[1])
                print("current heading is", m.degrees(turtlebot_evader.orientation_euler[2]))
                print("desired heading is", m.degrees(desired_heading),"heading error is", heading_error)
        
                if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                    turtlebot_evader.move_turtle(line_speed, stop_speed)
                elif abs(dist_error) < turn_tolerance and  abs(heading_error) >= heading_tol:
                    if heading_error >= 0:
                        turtlebot_evader.move_turtle(stop_speed, turn_speed)
                        #turtlebot_evader.move_turtle(line_speed, (turn_speed))
                    else:
                        turtlebot_evader.move_turtle(stop_speed, (-1.0*turn_speed))
                        #turtlebot_evader.move_turtle(line_speed, (0.0))
                else:

                        turtlebot_evader.move_turtle(line_speed, (turn_speed))       
                        #turtlebot_evader.move_turtle(line_speed, (0.0))         
                desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

                
                rclpy.spin_once(turtlebot_evader)
                                
        #/we're done looping through our lists
        is_done = True
        del(actual_path_x[0])
        del(actual_path_y[0])

        ax.plot(actual_path_x, actual_path_y, 'b')
        ax.plot(pursuer_path_x, pursuer_path_y, 'y')
                        

if __name__=="__main__":
    main()