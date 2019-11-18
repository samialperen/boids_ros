#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from util import Vector2, angle_diff


def get_agent_velocity(agent):
    """Return agent velocity as Vector2 instance."""
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel


def get_agent_position(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2() 
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos

def get_leader_position(leader):
    """Return obstacle position as Vector2 instance."""
    pos = Vector2()
    pos.x = leader.position.x
    pos.y = leader.position.y
    return pos


def get_obst_position(obst):
    """Return obstacle position as Vector2 instance."""
    pos = Vector2()
    pos.x = obst.position.x
    pos.y = obst.position.y
    return pos


class Boid(object):
    """
    An implementation of Craig Reynolds' flocking rules and boid objects.

    Each boid (bird-oid object) maneuvers based on the positions and velocities
    of its nearby flockmates. Computation is based on three components:
    1) alignment: steer towards the average heading of local flockmates
    2) cohesion: steer to move toward the average position of local flockmates
    3) separation: steer to avoid crowding local flockmates.
    Additionally, 4th component, avoid, is implemented where boids steer away
    from obstacles in their search radius.

    Each component yields a force on boid. Total force then gives the
    acceleration which is integrated to boid's velocity. Force and velocity are
    limited to the specified amount.

    State:
        position (Vector2): Boid's position
        velocity (Vector2): Boid's velocity

    Parameters:
        mass (double): Boid's mass
        alignment_factor (double): Weight for alignment component
        cohesion_factor (double): Weight for cohesion component
        separation_factor (double): Weight for separation component
        avoid_factor (double): Weight for obstacle avoiding component
        max_speed (double): Velocity upper limit
        max_force (double): Force upper limit
        friction (double): Constant friction force
        crowd_radius (double): Radius to avoid crowding
        search_radius (double): Boid's sensing radius
        avoid_radius (double): Radius to avoid obstacles

    Methods:
        update_parameters(self): Save parameters in class variables
        compute_alignment(self, nearest_agents): Return alignment component
        compute_cohesion(self, nearest_agents): Return cohesion component
        compute_separation(self, nearest_agents): Return separation component
        compute_avoids(self, avoids): Return avoid component
        compute_velocity(self, my_agent, nearest_agents, avoids):
            Compute total velocity based on all components
    """

    def __init__(self, initial_velocity_x, initial_velocity_y, wait_count, start_count, frequency):
        """Create an empty boid and update parameters."""
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

    
    

    def compute_avoids(self, avoids):
        """
        Return avoid component.

        This rule consists of two components. The first is active for all
        obstacles within range and depends on agent's distance from obstacle as
        well as its aproach angle. Force is maximum when agent approaches the
        obstacle head-on and minimum if obstacle is to the side of an agent.
        Second component depends only on distance and only obstacles closer than
        avoid_radius are taken into account. This is to ensure minimal distance
        from obstacles at all times.
        """
        main_direction = Vector2()
        safety_direction = Vector2()
        count = 0

        # Calculate repulsive force for each obstacle in sight.
        for obst in avoids:
            obst_position = get_obst_position(obst)
            d = obst_position.norm()
            obst_position *= -1        # Make vector point away from obstacle.
            obst_position.normalize()  # Normalize to get only direction.
            # Additionally, if obstacle is very close...
            if d < self.avoid_radius:
                # Scale lineary so that there is no force when agent is on the
                # edge of minimum avoiding distance and force is maximum if the
                # distance from the obstacle is zero.
                safety_scaling = -2 * self.max_force / self.avoid_radius * d + 2 * self.max_force
                safety_direction += obst_position * safety_scaling
                count += 1

            # For all other obstacles: scale with inverse square law.
            obst_position = obst_position / (d**2)
            main_direction += obst_position

        if avoids:
            # Calculate the approach vector.
            a = angle_diff(self.old_heading, main_direction.arg() + 180)
            # We mustn't allow scaling to be negative.
            side_scaling = max(math.cos(math.radians(a)), 0)
            # Divide by number of obstacles to get average.
            main_direction = main_direction / len(avoids) * side_scaling
            safety_direction /= count

        rospy.logdebug("avoids*:      %s", main_direction)
        # Final force is sum of two componets.
        # Force is not limited so this rule has highest priority.
        return main_direction + safety_direction

     
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

      
    def compute_velocity(self, my_agent, nearest_agents, avoids,rel2leader):
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
            avoid = self.compute_avoids(avoids) 

            leader = self.compute_leader_following(rel2leader)
            

            # Add components together and limit the output.
            force = Vector2()
            #force += v1 * self.rule1_weight
            #force += v2 * self.rule2_weight
            #force += v3 * self.rule3_weight
            #force += avoid * self.obstacle_weight
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
            self.viz_components['avoid'] = avoid * self.obstacle_weight
            self.viz_components['leader'] = leader * self.leader_weight
            self.viz_components['acceleration'] = acceleration / self.frequency
            self.viz_components['velocity'] = self.velocity
            self.viz_components['estimated'] = self.old_velocity
            return vel, self.viz_components
