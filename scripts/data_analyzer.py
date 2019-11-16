"""
This python script reads all agents odometry and velocity from bag files
and calculates necessary metrics to quantify boids algorithm.
"""

import rosbag
import rospy
import numpy as np
import pandas as pd


# This function is used to measure euclidian distance between pd dataframes
def get_distance(a, b):
    # a, b --> pandas dataframes, inputs
    # output --> difference panda dataframe which contains
    # eucledian distances for all times
    difference = pd.DataFrame(columns=['distance','t'])
    difference['t'] = a['t']
    x_dif_square = np.square(a['x']-b['x'])
    y_dif_square = np.square(a['y']-b['y'])
    difference['distance'] = np.sqrt(x_dif_square + y_dif_square)
    return difference


#################### Parameters #########################################
seperation_threshold = 2.0
#total_num_of_robots = rospy.get_param("/num_of_robots")
total_num_of_robots = 13


#################### Read Bag File ######################################
bag = rosbag.Bag('../bagfiles/example4.bag') #Read bag

############## General parameters obtained from rosbag
# The data between start_time and end_time will be analyzed
start_time = bag.get_start_time()  
end_time = bag.get_end_time() 
total_time = end_time - start_time

topics = bag.get_type_and_topic_info()[1].keys() #All topics in rosbag

############## Read poses 
# Read leader pose --> for our case only pose.x and pose.y is enough (2D)
df_leader_poses = pd.DataFrame(columns=['x','y','t'])
leader_counter = 0
for _ , msg, t in bag.read_messages("/robot_0/odom"):
        df_leader_poses.loc[leader_counter] = [msg.pose.pose.position.x, msg.pose.pose.position.y, t.to_sec()]
        leader_counter += 1

# Total number of pose msgs leader published, this will be used to synchorize boids
leader_pose_msg_size = df_leader_poses.shape[0] 

boids_poses = {} # Dictionary for all boids poses through time 
# Example: boids[1] contains all poses for robot_1
# boids[total_num_of_robots] contains all poses for robot_total_num_of_robots 
for robot_idx in range(1,total_num_of_robots): #start from robot_1
    boids_poses[robot_idx] = pd.DataFrame(columns=['x','y','t'])
    row_idx = 0
    for topic , msg, t in bag.read_messages("/robot_" + str(robot_idx) + "/odom"):
            boids_poses[robot_idx].loc[row_idx] = [msg.pose.pose.position.x, msg.pose.pose.position.y
                                             , t.to_sec()]
            row_idx += 1
    
    # This part is to make sure that all obtained data is synchorized
    if boids_poses[robot_idx].shape[0] > leader_pose_msg_size:
        d = boids_poses[robot_idx].shape[0] - leader_pose_msg_size #number of rows to delete
        # We are deleting first n rows i.e. we will take rows only starting from index n 
        boids_poses[robot_idx] = boids_poses[robot_idx].iloc[d:,]
    elif boids_poses[robot_idx].shape[0] < leader_pose_msg_size:
        a = leader_pose_msg_size - boids_poses[robot_idx].shape[0] #number of rows to add 
        line_to_be_added = boids_poses[robot_idx].iloc[0:1,] #first line of boids
        for i in range(a):
            boids_poses[robot_idx] = pd.concat([ line_to_be_added, boids_poses[robot_idx] ]).reset_index(drop = True) 
        boids_poses[robot_idx].iloc[0:a,2] = df_leader_poses.iloc[0:a,2] #change time

############## Read orientations
# Read leader orientation --> Since it is 2D, we need to subscribe cmd_vel
# arctan(cmd_vel.linear.y / cmd_vel.linear.x) will give the orientation, i.e. angle 
df_leader_angles = pd.DataFrame(columns=['angle','t'])
leader_counter = 0
for _ , msg, t in bag.read_messages("/robot_0/cmd_vel"):
        df_leader_angles.loc[leader_counter] = [np.degrees(np.arctan2(msg.linear.y,msg.linear.x))
                                                , t.to_sec()]
        leader_counter += 1

# Total number of pose msgs leader published, this will be used to synchorize boids
leader_angle_msg_size = df_leader_angles.shape[0] 

boids_angles = {} # Dictionary for all boids poses through time 
# Example: boids[1] contains all poses for robot_1
# boids[total_num_of_robots] contains all poses for robot_total_num_of_robots 
for robot_idx in range(1,total_num_of_robots): #start from robot_1
    boids_angles[robot_idx] = pd.DataFrame(columns=['angle','t'])
    row_idx = 0
    for topic , msg, t in bag.read_messages("/robot_" + str(robot_idx) + "/cmd_vel"):
            boids_angles[robot_idx].loc[row_idx] = [np.degrees(np.arctan2(msg.linear.y,msg.linear.x))
                                                    , t.to_sec()]
            row_idx += 1
    
    ## This part is to make sure that all obtained data is synchorized
    if boids_angles[robot_idx].shape[0] > leader_angle_msg_size:
        d = boids_angles[robot_idx].shape[0] - leader_angle_msg_size #number of rows to delete
        print(d)
        print(boids_angles[robot_idx]['t'][0])
        print(df_leader_angles['t'][0])
        for _ in range(d):
            print("inside for")
            if boids_angles[robot_idx]['t'][0] != df_leader_angles['t'][0]:
                # We are deleting first row
                print("delete first")
                boids_angles[robot_idx] = boids_angles[robot_idx].iloc[1:,].reset_index(drop=True)
            #    #boids_angles[robot_idx].drop(boids_angles[robot_idx].index[0])
            else:
                # We need to delete last row
                print("delete last")
                #total_row_number = boids_angles[robot_idx].shape[0] 
                boids_angles[robot_idx] = boids_angles[robot_idx][:-1]
                #boids_angles[robot_idx].drop(boids_angles[robot_idx].tail(1))
    elif boids_angles[robot_idx].shape[0] < leader_angle_msg_size:
        d = leader_angle_msg_size - boids_angles[robot_idx].shape[0] #number of rows to delete
        df_leader_angles = df_leader_angles.iloc[d:,].reset_index()
         
        

#print(df_leader_angles.shape)
#print(boids_angles[1].shape)
#print(boids_angles[2].shape)
#print(boids_angles[3].shape)
#print(boids_angles[4].shape)
#print(boids_angles[5].shape)
#print(boids_angles[6].shape)
#print(boids_angles[7].shape)
#print(boids_angles[8].shape)
#print(boids_angles[9].shape)
#print(boids_angles[10].shape)
#print(boids_angles[11].shape)
#print(boids_angles[12].shape)

print(df_leader_angles)
print(boids_angles[1])
print(boids_angles[2])
print(boids_angles[3])
print(boids_angles[4])
print(boids_angles[5])
print(boids_angles[6])
print(boids_angles[7])
print(boids_angles[8])
print(boids_angles[9])
print(boids_angles[10])
print(boids_angles[11])
print(boids_angles[12])






#################### Calculate Metrics ######################################
# Calculate relative distance to leader for each robot for all the time
boids_rel2leader_poses = {} # Dictionary for all boids distances to leader
# E.g. boids_rel2leader[2] will contain distance of robot_2 to leader for all the times
# boids_rel2leader[2] structure will be an pd dataframe with columns--> distance and t
for robot_idx in range(1,total_num_of_robots):
    boids_rel2leader_poses[robot_idx] = get_distance(df_leader_poses, boids_poses[robot_idx]) 




bag.close()