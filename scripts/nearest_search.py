#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
import rospy
import message_filters as mf
from copy import deepcopy

from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sphero_formation.msg import OdometryArray

class NearestSearch(object):
    """
    Node that provides information about nearest flockmates and obstacles.

    Generally, Reynolds' flocking algorithm works on distributed systems.
    If agents don't have any sensors, a centralized system is needed. This node
    is an 'all-knowing' hub that makes virtual distributed system possible.
    It is subscribed to messages with position and velocity of each agent and
    knows the map layout. For each agent, it finds its neighbors within search
    radius and calculates their relative position. This data is then published
    to individual agents along side the list of obstacles within range.
    """

    def map_callback(self, data):
        """Save map occupancy grid and meta-data in class variables."""
        self.map = []
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_origin = data.info.origin.position

        # Reverse the order of rows in map
        for i in range(self.map_height - 1, -1, -1):
            self.map.append(data.data[i * self.map_width:(i + 1) * self.map_width])

    def pos_to_index(self, x_real, y_real):
        """Return list (map) indices for given real position coordinates."""
        col = (x_real - self.map_origin.x) / self.map_resolution
        row = (self.map_origin.y + self.map_height *
               self.map_resolution - y_real) / self.map_resolution
        return int(col), int(row)

    def index_to_pos(self, row, col):
        """Return real position coordinates for list (map) indices."""
        x_real = self.map_origin.x + col * self.map_resolution
        y_real = self.map_origin.y + (self.map_height - row) * self.map_resolution
        return x_real, y_real

    def param_callback(self, data):
        """Update search parameters from server."""
        while not rospy.has_param('/dyn_reconf/search_radius'):
            rospy.sleep(0.1)

        self.search_radius = rospy.get_param('/dyn_reconf/search_radius')
        self.r = int(self.search_radius / self.map_resolution)

    def robot_callback(self, *data):
        """
        This callback function is used to publish following topics:
            nearest_robots --> contains pose of nearby agents to each agent in flock
        e.g. /robot_1/robots topic contains pose of nearby agents to robot_1
            avoids --> contains pose of obstacles in the map relative to each agent
        e.g. /robot_2/avoids topic contains pose of obstacles rel. to robot_2
            rel_target --> contains pose of each agent relative to the leader (robot_0)
        e.g. /robot_6/rel2leader topic contains relative pose of robot_6 to leader
        Note: For simplicity, robot_0 is chosen as a leader!
        """

        for robot in data:
            time = rospy.Time.now() #Current time
            robot_name = robot.header.frame_id.split('/')[1] #robot name
            
            robot_position = robot.pose.pose.position #current robot's position
            
            ####################### Nearest Robots ###################################
            nearest_robots = OdometryArray() #collect pose of all nearby robots
            nearest_robots.header.stamp = time

            nearest_robots.array.append(deepcopy(robot)) # add current robot's odom to array

            # Now look for neighbor robots within horizon of each robot
            for neighbor in data:
                neighbor_position = neighbor.pose.pose.position
                # Distance between robot_position and neighbor_position
                d = math.sqrt(pow(robot_position.x - neighbor_position.x, 2)  
                              + pow(robot_position.y - neighbor_position.y, 2))
                if d > 0 and d <= self.search_radius:
                    rel_neighbor_pos = deepcopy(neighbor)
                    rel_neighbor_pos.pose.pose.position.x = neighbor_position.x - robot_position.x
                    rel_neighbor_pos.pose.pose.position.y = neighbor_position.y - robot_position.y
                    nearest_robots.array.append(rel_neighbor_pos)

            self.nearest[robot_name].publish(nearest_robots) #Send to ros publisher
            
            #########################################################################


            ####################### Obstacles ###################################
            # Obstacle avoidence part is derived from:
            # https://github.com/mkrizmancic/sphero_formation/blob/master/scripts/nearest_search.py
            obstacles = PoseArray()
            obstacles.header.stamp = time

            # Positions of walls and obstacles are represented as value 100 in
            # the list `self.map`. First, find the position of the observed
            # agent in this list in form of an index pair. Then search the list
            # in specified search radius and return actual positions of the
            # walls and other obstacles.
            or_col, or_row = self.pos_to_index(robot_position.x, robot_position.y)
            # if key == 'sphero_0': print (or_col, or_row)
            col_range = range(max(0, or_col - self.r), min(self.map_width, or_col + self.r + 1))
            row_range = range(max(0, or_row - self.r), min(self.map_height, or_row + self.r + 1))
            for row in row_range:
                for col in col_range:
                    # if key == 'sphero_0': print(self.map[row][col]/100, end=' ')
                    # Check only elements within radius.
                    # Search obstacles in a circle instead of a square.
                    if (pow(row - or_row, 2) + pow(col - or_col, 2)) <= pow(self.r, 2):
                        if self.map[row][col] == 100:
                            x, y = self.index_to_pos(row, col)
                            obst = Pose()
                            obst.position.x = x - robot_position.x
                            obst.position.y = y - robot_position.y
                            obstacles.poses.append(obst)
                
            self.avoid[robot_name].publish(obstacles)
            #########################################################################
       
            ############################## Leader ###############################           
            leader = Pose() #contains pose of leader
            rel_target = PoseArray() #contains pose of current agent rel to leader
            if robot_name == "robot_0": #leader
                leader.position = robot.pose.pose.position
            else:
                obst_target = Pose()
                obst_target.position.x = leader.position.x-robot_position.x
                obst_target.position.y = leader.position.y-robot_position.y
                # getting time to rel_target header is important to synchorize this topic
                # with other ROS topics (e.g. .../obstacles .../nearest_robots)
                rel_target.header.stamp = time #this is important to synchorize
                rel_target.poses.append(obst_target)
                self.leader[robot_name].publish(rel_target)
            ##################################################################

            
    def __init__(self):
        """Create subscribers and publishers."""

        # Get parameters and initialize class variables.
        self.num_agents = rospy.get_param('/num_of_robots')
        robot_name = rospy.get_param('~robot_name')
        
        # Create publishers for commands
        pub_keys = [robot_name + '_{}'.format(i) for i in range(self.num_agents)]

        # Publisher for locations of nearest agents
        self.nearest = dict.fromkeys(pub_keys)
        for key in self.nearest.keys():
            self.nearest[key] = rospy.Publisher('/' + key + '/nearest', OdometryArray, queue_size=1)

        # Publisher for locations of walls and obstacles
        self.avoid = dict.fromkeys(pub_keys)
        for key in self.avoid.keys():
            self.avoid[key] = rospy.Publisher('/' + key + '/avoid', PoseArray, queue_size=1)

        # Publisher for relative position to leader
        self.leader = dict.fromkeys(pub_keys)
        for key in self.leader.keys():
            # robot_0 is leader, so no need to publish it's relative position to itself :)
            if key != "robot_0": 
                self.leader[key] = rospy.Publisher('/' + key + '/rel2leader', PoseArray, queue_size=1)

        # Create subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.sleep(0.5)  # Wait for first map_callback to finish
        rospy.Subscriber('/dyn_reconf/parameter_updates', Config, self.param_callback, queue_size=1)
        self.param_callback(None)
     
        topic_name = '/' + robot_name + '_{}/odom'

        subs = [mf.Subscriber(topic_name.format(i), Odometry) for i in range(self.num_agents)]
        self.ts = mf.ApproximateTimeSynchronizer(subs, 10, 0.11) 
        self.ts.registerCallback(self.robot_callback)

        rospy.spin() 


if __name__ == '__main__':
    rospy.init_node('NearestSearch')

    try:
        ns = NearestSearch()
    except rospy.ROSInterruptException:
        pass
