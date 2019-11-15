#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

class control_leader:
    
    def __init__(self):
        rospy.init_node('Leader Controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/robot_0/odom', Odom, self.update_leader_pose)
        
        self.speed = rospy.get_param("/dyn_reconf/max_speed")
    
    def update_leader_pose(self,data):
        self.pose = Pose()
        self.pose = data.pose
        
    def euclidean_distance(self, goal_pose):       
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
    
    def linear_vel(self, goal_pose):
        return self.speed * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose):
        return self.speed * (self.steering_angle(goal_pose) - self.pose.theta)
    
    
    def go_desired_pose(self,desired_pose):
        start_pose = Pose()
        start_pose.x = desired_pose.x
        start_pose.y = desired_pose.y

        error_margin = 0.01
        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= error_margin:
            vel_msg.linear.x = self.linear_vel(start_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(start_pose)

    def draw_square(self):
        start_pose = self.pose
        start_pose.x = 0.0
        start_pose.y = 3.0
        if self.pose.x != start_pose.x and self.pose.y != start_pose.y:
            go_desired_pose(start_pose) #(0,3) starting position
 




if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass