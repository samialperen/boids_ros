#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from util import Vector2, angle_diff


class Boid(object):
    def __init__(self, initial_velocity_x, initial_velocity_y, wait_count, start_count, frequency):
        self.position = Vector2()
        self.velocity = Vector2()
        self.mass = 0.18                # Mass of Sphero robot in kilograms
        self.wait_count = wait_count    # Waiting time before starting
        self.start_count = start_count  # Time during which initial velocity is being sent
        self.frequency = frequency      # Control loop frequency

        # Set initial velocity
        self.initial_velocity = Twist()
        self.initial_velocity.linear.x = initial_velocity_x
        self.initial_velocity.linear.y = initial_velocity_y

        # This dictionary holds values of each flocking components and is used
        # to pass them to the visualization markers publisher.
        self.viz_components = {}

    def update_parameters(self, params):
        self.rule1_weight = params['cohesion_weight']
        self.rule2_weight = params['separation_weight']
        self.rule3_weight = params['alignment_weight']
        self.obstacle_weight = params['obstacle_weight']
        #self.leader_weight = 1.0
        self.leader_weight = params['leader_weight']
        self.max_speed = params['max_speed']
        self.max_force = params['max_force']
        self.friction = params['friction']
        self.desired_separation = params['desired_separation']
        self.horizon = params['horizon']
        self.avoid_radius = params['avoid_radius']
     

    def rule1(self, nearest_agents): #Cohesion
        center_of_mass = Vector2()        
        com_direction = Vector2()
        # Find mean position of neighboring agents.
        for b in nearest_agents:
            boid_position = get_agent_position(b)
            center_of_mass += boid_position

        # Magnitude of force is proportional to agents' distance 
        # from the center of mass.
        # Force should be applied in the direction of com
        if nearest_agents:
            com_direction = center_of_mass / len(nearest_agents)
            #rospy.logdebug("cohesion*:    %s", direction)
            d = com_direction.norm()
            com_direction.set_mag((self.max_force * (d / self.horizon)))
        
        return com_direction

    def rule2(self, nearest_agents): #Seperation
        
        c = Vector2()
        N = 0 #Total boid number

        for b in nearest_agents:
            boid_position = get_agent_position(b)
            d = boid_position.norm()
            if d < self.desired_separation:
                N += 1
                boid_position *= -1        # Force towards outside
                boid_position.normalize()  # Normalize to get only direction.
                # magnitude is proportional to inverse square of d
                # where d is the distance between agents
                boid_position = boid_position / (d**2)
                c += boid_position

        if N:            
            c /= N #average
            c.limit(2 * self.max_force)  # 2 * max_force gives this rule a slight priority.
        
        return c
    
    def rule3(self, nearest_agents): #Alignment
        perceived_velocity = Vector2()
        pv = Vector2()
        # Find mean direction of neighboring agents.
        for boid in nearest_agents:
            boid_velocity = get_agent_velocity(boid)
            perceived_velocity += boid_velocity #mean perceived 

        # Steer toward calculated mean direction with maximum velocity.
        if nearest_agents:
            perceived_velocity.set_mag(self.max_speed)
            pv = perceived_velocity - self.velocity
            pv.limit(self.max_force)
        return pv

    
    def compute_leader_following(self,rel2leader):
        for agent in rel2leader:
            rel2leader_position = get_leader_position(agent)
            # Force in the direction that minimizes rel_position 2 leader
            # i.e. it should be in the direction of rel2leader
            direction = Vector2() #initiliazes (0,0)
            direction = rel2leader_position # *0.01
            # d = direction.norm()
            # direction.set_mag((self.max_force * d))
        
        return direction

      
    def compute_velocity(self, my_agent, nearest_agents,rel2leader):
        """Compute total velocity based on all components."""

        # While waiting to start, send zero velocity and decrease counter.
        if self.wait_count > 0:
            self.wait_count -= 1
            rospy.logdebug("wait " + '{}'.format(self.wait_count))
            rospy.logdebug("velocity:\n%s", Twist().linear)
            return Twist(), None

        # Send initial velocity and decrease counter.
        elif self.start_count > 0:
            self.start_count -= 1
            rospy.logdebug("start " + '{}'.format(self.start_count))
            rospy.logdebug("velocity:\n%s", self.initial_velocity.linear)
            return self.initial_velocity, None

        # Normal operation, velocity is determined using Reynolds' rules.
        else:
            self.velocity = get_agent_velocity(my_agent)
            self.old_heading = self.velocity.arg()
            self.old_velocity = Vector2(self.velocity.x, self.velocity.y)
            rospy.logdebug("old_velocity: %s", self.velocity)

            # Compute all the components.
            v1 = self.rule1(nearest_agents) #cohesion
            v2 = self.rule2(nearest_agents) #seperation
            v3 = self.rule3(nearest_agents) #alignment

            leader = self.compute_leader_following(rel2leader)
            

            # Add components together and limit the output.
            force = Vector2()
            force += v1 * self.rule1_weight
            force += v2 * self.rule2_weight
            force += v3 * self.rule3_weight
            force += leader * self.leader_weight
            
            force.limit(self.max_force)

            # If agent is moving, apply constant friction force.
            # If agent's velocity is less then friction / 2, it would get
            # negative velocity. In this case, just stop it.
            #if self.velocity.norm() > self.friction / 2:
            #    force += self.friction * -1 * self.velocity.normalize(ret=True)
            #else:
            #    self.velocity = Vector2()

            acceleration = force / self.mass

            # Calculate total velocity (delta_velocity = acceleration * delta_time).
            self.velocity += acceleration / self.frequency
            self.velocity.limit(self.max_speed)

            #rospy.logdebug("force:        %s", force)
            #rospy.logdebug("acceleration: %s", acceleration / self.frequency)
            #rospy.logdebug("velocity:     %s\n", self.velocity)

            # Return the the velocity as Twist message.
            vel = Twist()
            vel.linear.x = self.velocity.x
            vel.linear.y = self.velocity.y

            # Pack all components for Rviz visualization.
            # Make sure these keys are the same as the ones in `util.py`.
            self.viz_components['cohesion'] = v1 * self.rule1_weight
            self.viz_components['separation'] = v2 * self.rule2_weight
            self.viz_components['alignment'] = v3 * self.rule3_weight
            #self.viz_components['avoid'] = avoid * self.obstacle_weight
            self.viz_components['leader'] = leader * self.leader_weight
            #self.viz_components['acceleration'] = acceleration / self.frequency
            #self.viz_components['velocity'] = self.velocity
            #self.viz_components['estimated'] = self.old_velocity
            return vel, self.viz_components

def get_agent_velocity(agent):
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel


def get_agent_position(agent):
    pos = Vector2() 
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos

def get_leader_position(leader):
    pos = Vector2()
    pos.x = leader.position.x
    pos.y = leader.position.y
    return pos


def get_obst_position(obst):
    pos = Vector2()
    pos.x = obst.position.x
    pos.y = obst.position.y
    return pos