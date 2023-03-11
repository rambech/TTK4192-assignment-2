#!/usr/bin/env python3

import rospy
import random
import math 
from time import time

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

def nearest(vertex_list, new_vertex):
    prev_dist = math.inf
    nearest_vertex = None

    for vertex in vertex_list:
        dist = math.dist(vertex, new_vertex)

        if dist < prev_dist:
            nearest_vertex = vertex
            prev_dist = dist

    return nearest_vertex

def nearest_obstacle(nearest_vertex, new_vertex, obstacle_radius):
    '''
    Shortes the a branch to fit within obstacles
    '''
    dist = math.dist(nearest_vertex, new_vertex)
    angle = math.atan2(new_vertex[1] - nearest_vertex[1], new_vertex[0] - nearest_vertex[0])

    while(isThruObstacle(nearest_vertex, new_vertex, obstacle_radius).throughObstacle or isInObstacle(new_vertex, obstacle_radius).inObstacle):
          dist -= 0.1
          new_vertex = (dist*math.cos(angle), dist*math.sin(angle))

          if dist < 0:
              break

    return dist, new_vertex

def backtrack(vertex_list, connected_list):
    list_of_positions = []
    prev_vertex = vertex_list[-1]
    list_of_positions.insert(0, prev_vertex)

    # Would be cool to get the path in another colour,
    # but that is a task for another time
    # path_marker_color = [0/256, 158/256, 115/256]

    while (True):
        if (vertex_list.index(prev_vertex) == 0):
            break

        for connection in connected_list:
            if connection[1] == vertex_list.index(prev_vertex): # Something like that...
                prev_vertex = vertex_list[connection[0]]
                list_of_positions.insert(0, prev_vertex)
                break

    return list_of_positions



def get_edge_as_marker(first_point, second_point, color, identity, thickness=0.025):

    edge_marker = get_marker(Marker.LINE_STRIP, [0,0], thickness, color, identity)
    
    p0_point = Point(first_point[0], first_point[1], 0.0)
    p1_point = Point(second_point[0], second_point[1], 0.0)
    edge_marker.points.append(p0_point)
    edge_marker.points.append(p1_point)
    
    return edge_marker

def RRT(startpos, endpos, step_length = 1, goal_radius=0.1):
    '''
    My implementation of the RRT algorithm with visualization
    '''

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
    vertex_list.append(startpos)

    connected_list = []

    num_iter = 0    
    obstacle_radius = 0.3
    dist = 0

    while (True):
            # Choosing random vertex to branch from
            parent_vertex = random.choice(vertex_list)
            new_vertex_direction = 2*math.pi*random.random()
            new_vertex = (parent_vertex[0] + step_length*math.cos(new_vertex_direction), parent_vertex[1] + step_length*math.sin(new_vertex_direction))

            num_iter += 1

            if (num_iter % 10 == 0):
                rospy.Rate(0.5).sleep() # needs to wait a bit for the publisher to start, a bit weird. 
                tree_publisher.publish(tree_marker)
                print(f"Iteration {num_iter}")
            
            obs_response = isInObstacle(new_vertex, obstacle_radius).inObstacle
            # If vertex in obstacle go to next loop
            if (obs_response):
               continue
           
            nearest_vertex = nearest(vertex_list, new_vertex)

            # if (isInObstacle(new_vertex, obstacle_radius).inObstacle or isThruObstacle(nearest_vertex, new_vertex, obstacle_radius).throughObstacle):
            #     dist, new_vertex = nearest_obstacle(nearest_vertex, new_vertex, obstacle_radius)

            # if dist < 0:
            #     continue

            thru_response = isThruObstacle(nearest_vertex, new_vertex, obstacle_radius).throughObstacle
            # Go to next loop if line crosses through obstacle
            if (thru_response):
               continue

            edge_marker = get_edge_as_marker(nearest_vertex, new_vertex, edge_color, marker_identity)
            marker_identity += 1
            tree_marker.markers.append(edge_marker)

            # Add vertex to list, and indices to list of connection
            vertex_list.append(new_vertex)
            connected_list.append((vertex_list.index(nearest_vertex), len(vertex_list) - 1))
            print(f"Connection made: {connected_list[-1]}")

            # End condition
            if (math.dist(new_vertex, endpos) < goal_radius):
                break

    
    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)

    return num_iter, connected_list, vertex_list

def traverse(pos_list):
    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in pos_list:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)




if __name__ == '__main__':
    # -----------------
    # Init the RRT node
    rospy.init_node('RRT')



    # -----------------------------------------------
    # The start and end positions of the "short maze"
    # startpos = (0.0, 0.0)
    # endpos = (4.5, 5.0)

    # -------------------------------------------------------------------------
    # The start and end positions of the bonus task with the "complicated maze"
    startpos = (0.0, 0.0)
    endpos = (4.5, 9.0)


    start_time = time()

    num_iter, connected_list, vertex_list = RRT(startpos, endpos, 2, 0.5)
    
    end_time = time()

    print("---------- RRT Done ----------")
    print(f"Finished in {num_iter} iterations")
    print(f"Finished in {end_time - start_time} seconds")
    
    list_of_positions = backtrack(vertex_list, connected_list)

    traverse(list_of_positions)
