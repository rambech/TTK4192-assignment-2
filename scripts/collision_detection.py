#!/usr/bin/env python3

import rospy
import numpy as np
from shapely.geometry import LineString, Point, MultiPoint

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleResponse, isInObstacle, isInObstacleResponse

class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist
        self.p0 = p0
        self.p1 = p1

    def path(self, t):
        return self.p + t * self.dirn

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def Intersection(line, center, radius):
    p = Point(center[0],center[1])
    c = p.buffer(radius).boundary
    l = LineString([line.p0, line.p1])
    return c.intersects(l)

def Intersection_multi(line, obstacles, radius):
    
    l = LineString([line.p0, line.p1])
    return obstacles.intersects(l)

class CollisionDetector:
  
    def __init__(self):

        def callback(msg):
            self.map = msg
            self.new_map = True

        self.new_map = False
        rospy.Subscriber("/map", OccupancyGrid, callback)

        try: 
            self.map = OccupancyGrid
        except NameError:
            print("NameError: ")
            print("OccupancyGrid is not yet imported...")

        try:
            self.path_through_obstacle_service = rospy.Service('path_through_obstacle', isThroughObstacle, self.check_is_through_obstacle)
            self.point_in_obstacle_service = rospy.Service('point_in_obstacle', isInObstacle, self.check_is_in_obstacle)
        
        except NameError:
            print("NameError: ")
            print("isThroughObstacle and isInObstacle is not yet defined... Will be done in 2.c and 2.d")
        self.obstacle_radius = 0.28

        self.obstacle_pub = rospy.Publisher('obstacle_marker', MarkerArray, queue_size=10)

        self.obstacles = []
        self.obstacle_objects = None


    def create_obstacles_from_map(self):
        obstacle_map = self.map.data

        obstacle_map = np.reshape(obstacle_map, (self.map.info.height, self.map.info.width))
        self.obstacles = []
        for i in range(self.map.info.height):
            for j in range(self.map.info.width):
                if obstacle_map[i][j] > 50:
                    self.obstacles.append([j*self.map.info.resolution+self.map.info.origin.position.x, i*self.map.info.resolution+self.map.info.origin.position.y])

        self.obstacle_objects = MultiPoint(self.obstacles).buffer(self.obstacle_radius).simplify(0.1).boundary    

    def check_is_in_obstacle(self, request):
        self.obstacle_radius = request.radius
        vex = [request.vex.x, request.vex.y]

        for obs in self.obstacles:
            if distance(obs, vex) < request.radius:
                return isInObstacleResponse(True)
        return isInObstacleResponse(False)


    def check_is_through_obstacle(self, request):
        self.obstacle_radius = request.radius

        p0 = [request.p0.x, request.p0.y]
        p1 = [request.p1.x, request.p1.y]

        line = Line(p0, p1)

        if Intersection_multi(line, self.obstacle_objects, request.radius):
            return isThroughObstacleResponse(True)


        return isThroughObstacleResponse(False)

    def draw_obstacles(self):

        obstacle_map = self.map.data

        obstacle_map = np.reshape(obstacle_map, (self.map.info.height, self.map.info.width))

        marker_arr = MarkerArray()

        identity = 0


        for i in range(self.map.info.height):
            for j in range(self.map.info.width):

                if obstacle_map[i][j] > 50:
                    identity += 1 
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.type = Marker.SPHERE
                    marker.id = identity
                    marker.pose.position.x = (0.5+j)*self.map.info.resolution+self.map.info.origin.position.x
                    marker.pose.position.y = (0.5+i)*self.map.info.resolution+self.map.info.origin.position.y
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.action = Marker.ADD

                    marker.scale.x = self.obstacle_radius
                    marker.scale.y = self.obstacle_radius
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    

                    marker_arr.markers.append(marker)


        self.obstacle_pub.publish(marker_arr)
        self.new_map = False


rospy.init_node('collision_detector')


collision_detector = CollisionDetector()

rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    rate.sleep()
    collision_detector.draw_obstacles()
    collision_detector.create_obstacles_from_map()