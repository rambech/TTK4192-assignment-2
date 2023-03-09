#!/usr/bin/env python3

import rospy
from random import random
from numpy.linalg import norm
from math import inf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleRequest, isInObstacle, isInObstacleRequest, positionControl, positionControlRequest


point_in_obstacle_service = rospy.ServiceProxy('point_in_obstacle', isInObstacle)

def isInObstacle(vex, radius): # This function name seems like a sketchy and semi-dangerous ambiguity 

    vex_pos = Point(vex[0], vex[1], 0.0)

    request = isInObstacleRequest(vex_pos, radius)
    response = point_in_obstacle_service(request)

    return response



path_through_obstacle_service = rospy.ServiceProxy('path_through_obstacle', isThroughObstacle)

def isThruObstacle(p0, p1, radius):

    p0_pos = Point(p0[0], p0[1], 0.0)
    p1_pos = Point(p1[0], p1[1], 0.0)

    request = isThroughObstacleRequest(p0_pos, p1_pos, radius)
    response = path_through_obstacle_service(request)

    return response

def get_marker(type, pos, size, color, identity):

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = type
    marker.id = identity
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.action = Marker.ADD

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = 0.001
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker

def nearest(vertex_list, goal_vertex):
    prev_dist = inf
    vex = None

    for i, vertex in enumerate(vertex_list):
        dist = norm((vertex[0], vertex[1]) - (goal_vertex[0], goal_vertex[1]))

        if dist < prev_dist:
            vex = vertex

        prev_dist = dist

    return vex
        

def get_edge_as_marker(first_point, second_point, color, identity, thickness=0.025):

    edge_marker = get_marker(Marker.LINE_STRIP, [0,0], thickness, color, identity)
    
    p0_point = Point(first_point[0], first_point[1], 0.0)
    p1_point = Point(second_point[0], second_point[1], 0.0)
    edge_marker.points.append(p0_point)
    edge_marker.points.append(p1_point)
    
    return edge_marker

def RRT(startpos, endpos):
    # RRT algorithm

    # Making markes on the map in real time
    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()
    marker_identity = 0

    # Create a blue-green square representing the start
    start_rgb_color = [0/256, 158/256, 115/256]
    start_marker_size = 0.2
    start_marker = get_marker(Marker.CUBE, startpos, start_marker_size, start_rgb_color, marker_identity)
    marker_identity += 1

    # Create a vermillion square representing the goal
    end_rgb_color = [213/256, 94/256, 0/256]
    end_marker_size = 0.2
    end_marker = get_marker(Marker.CUBE, endpos, end_marker_size, end_rgb_color, marker_identity)
    marker_identity +=1

    tree_marker.markers.append(start_marker)
    tree_marker.markers.append(end_marker)

    edge_color = [204/256, 121/256, 167/256]

    vertex_list = []
    vertex_list.append(startpos) # The visualizer might not accept tuples

    while (True):
            random_vertex = Point(3*random(), 3*random(), 0.0)
            obstacle_radius = 0.3

            # End condition
            if (norm((random_vertex[0], random_vertex[0]) - endpos) < obstacle_radius):
                vertex_list.append(random_vertex)
                break

            # If vertex in obstacle go to next loop
            if (isInObstacle(random_vertex, obstacle_radius)):
               continue
           
            nearest_vertex = nearest(vertex_list, random_vertex)

            # Go to next loop if line crosses through obstacle
            if (isThruObstacle(nearest_vertex, random_vertex)):
               continue

            vertex_list.append(random_vertex)

            edge_marker = get_edge_as_marker(nearest_vertex, random_vertex, edge_color, marker_identity)
            marker_identity += 1
            tree_marker.markers.append(edge_marker)


    # Testing publishing markers outside the loop first, 
    # would be cool to do inside the loop afterwards
    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)





    

if __name__ == '__main__':
    # -----------------
    # Init the RRT node
    rospy.init_node('RRT')



    # -----------------------------------------------
    # The start and end positions of the "short maze"
    if (maze == "short_maze"):
        startpos = (0.0, 0.0)
        endpos = (4.5, 5.0)



    # -------------------------------------------------------------------------
    # The start and end positions of the bonus task with the "complicated maze"
    if (maze == "complicated_maze"):
        startpos = (0.0, 0.0)
        endpos = (4.5, 9.0)


    
    # ---------------------------------------------------------------
    # Example of how you can check if a point is inside an obstacle: 

    obstacle_radius = 0.3
    vex_pos = [0.0, 0.0]
    response = isInObstacle(vex_pos, obstacle_radius)
    print("point_in_obstacle_service response: ")
    print(response)


    
    # -----------------------------------------------------------------------
    # Example of how you can check if the straightline path between two points 
    # goes through an obstacle:

    obstacle_radius = 0.3
    first_point = [0.0, 0.0]
    second_point = [1.0, 1.0]
    response = isThruObstacle(first_point, second_point, obstacle_radius)
    print("path_through_obstacle_service response: ")
    print(response)



    # -----------------------------------------------------------------------
    # Example of how you can visualize a graph using a MarkerArray publisher:

    list_of_positions = [[0.0, 0.0], [0.0, 2.0], [1.0, 2.0], [1.0, 1.0]]

    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()
    marker_identity = 0

    # Create a blue-green square representing the start
    start_rgb_color = [0/256, 158/256, 115/256]
    start_marker_size = 0.2
    start_marker = get_marker(Marker.CUBE, list_of_positions[0], start_marker_size, start_rgb_color, marker_identity)
    marker_identity += 1

    # Create a vermillion square representing the goal
    end_rgb_color = [213/256, 94/256, 0/256]
    end_marker_size = 0.2
    end_marker = get_marker(Marker.CUBE, list_of_positions[-1], end_marker_size, end_rgb_color, marker_identity)
    marker_identity +=1

    tree_marker.markers.append(start_marker)
    tree_marker.markers.append(end_marker)

    
    # Create reddish purple edges 
    edge_color = [204/256, 121/256, 167/256]
    
    for index in range(len(list_of_positions)-1):
        first_point = list_of_positions[index]
        second_point = list_of_positions[index+1]
        edge_marker = get_edge_as_marker(first_point, second_point, edge_color, marker_identity)
        marker_identity += 1
        tree_marker.markers.append(edge_marker)


    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)



    # ---------------------------------------------------------------------------
    # Example of how you can make the turtlebot go through a sequence of positions
    # using the position controller.

    # Note that it will just move in a straight line between the current position and the
    # desired position, so it will crash into possible obstacles. 
    # It is also not very well tuned (and quite slow).


    list_of_position = [[0.0, 2.0], [1.0, 2.0], [1.0, 1.0]]
    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in list_of_position:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)