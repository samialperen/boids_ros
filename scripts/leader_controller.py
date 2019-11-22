#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt

class Leader:

    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)

        rospy.init_node('leader_controller', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/robot_0/odom', Odometry, self.update_leader_pose)
        
        #self.speed = rospy.get_param("/dyn_reconf/max_speed")
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_leader_pose(self,data):
        self.pose = data.pose.pose
        
    def calculate_distance(self, goal_pose):       
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) 
                    + pow((goal_pose.position.y - self.pose.position.y), 2))
    
    def linear_vel(self, goal_pose):
        Kp_lin = 1.5
        return Kp_lin * self.calculate_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.pose.position.y, 
                     goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose):
        Kp_ang = 6 
        current_angle = atan2(self.pose.position.y,self.pose.position.x)
        return Kp_ang * (self.steering_angle(goal_pose) - current_angle)
        
    def go_desired_pose(self,desired_pose):
        target_pose = Pose()
        target_pose.position.x = 0.0 #desired_pose.position.x
        target_pose.position.y = 3.0 #desired_pose.position.y

        error_margin = 0.1
        vel_msg = Twist()
        while self.calculate_distance(target_pose) >= error_margin:
            vel_msg.linear.x = self.linear_vel(target_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
  
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(target_pose)

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep() #publish at the desired rate


        #Stop the leader after movement is done
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()


    def move_forward(self,amount,speed):
        target_pose = Pose()
        target_pose.position.x = self.pose.position.x #desired_pose.position.x
        target_pose.position.y = amount #desired_pose.position.y

        error_margin = 0.1
        vel_msg = Twist()
        while self.calculate_distance(target_pose) >= error_margin:
            vel_msg.linear.x = 0
            vel_msg.linear.y = speed
            vel_msg.linear.z = 0
  
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep() #publish at the desired rate


        #Stop the leader after movement is done
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)

        #rospy.spin()


    def move_backward(self,amount,speed):
        target_pose = Pose()
        target_pose.position.x = self.pose.position.x #desired_pose.position.x
        target_pose.position.y = -amount #desired_pose.position.y

        error_margin = 0.1
        vel_msg = Twist()
        while self.calculate_distance(target_pose) >= error_margin:
            vel_msg.linear.x = 0
            vel_msg.linear.y = -speed
            vel_msg.linear.z = 0
  
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep() #publish at the desired rate


        #Stop the leader after movement is done
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)

        #rospy.spin()


    def move_right(self,amount,speed):
        target_pose = Pose()
        target_pose.position.x = amount #desired_pose.position.x
        target_pose.position.y = self.pose.position.y #desired_pose.position.y

        error_margin = 0.1
        vel_msg = Twist()
        while self.calculate_distance(target_pose) >= error_margin:
            vel_msg.linear.x = speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
  
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep() #publish at the desired rate


        #Stop the leader after movement is done
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)

        #rospy.spin()

    def move_left(self,amount,speed):
        target_pose = Pose()
        target_pose.position.x = -amount #desired_pose.position.x
        target_pose.position.y = self.pose.position.y #desired_pose.position.y
        error_margin = 0.1
        vel_msg = Twist()
        while self.calculate_distance(target_pose) >= error_margin:           
            vel_msg.linear.x = -speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
  
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep() #publish at the desired rate


        #Stop the leader after movement is done
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)

        #rospy.spin()        
            

    def draw_square(self):
        speed = 0.55
        self.move_forward(4.0,speed)
        self.move_left(4.0,speed)
        self.move_backward(4.0,speed)
        self.move_right(4.0,speed)
        self.move_forward(4.0,speed)
        self.move_left(4.0,speed)
       
        rospy.spin()
       


if __name__ == '__main__':
    try:
        leader = Leader()
        leader.draw_square()
    except rospy.ROSInterruptException: pass