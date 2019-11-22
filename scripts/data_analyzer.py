"""
This python script reads all agents odometry and velocity from bag files
and calculates necessary metrics to quantify boids algorithm.
"""

import rosbag
import rospy
import numpy as np
import pandas as pd
import sys #for parser


# This function is used to measure euclidian distance between pd dataframes
def get_distance(a, b):
    # a, b --> pandas dataframes, inputs
    # output --> difference panda dataframe which contains
    # eucledian distances for all times
    distance = pd.DataFrame(columns=['distance','t'])
    distance['t'] = a['t']
    x_dif_square = np.square(a['x']-b['x'])
    y_dif_square = np.square(a['y']-b['y'])
    distance['distance'] = np.sqrt(x_dif_square + y_dif_square)
    return distance

# This function returns |a-b| 
def get_abs_difference(a, b):
    # a, b --> pandas dataframes, inputs
    # output --> |a-b|

    difference = pd.DataFrame(columns=['angle','t'])
    difference['t'] = a['t']
    difference['angle'] = abs(a['angle']-b['angle'])   
    return difference    


#################### Parameters #########################################
separation_threshold = 0.7 #in meters
alignment_threshold = 45.0 # degree
cohesion_threshold = 2.25  #meter
#total_num_of_robots = rospy.get_param("/num_of_robots")
total_num_of_robots = 19


#################### Read Bag File ######################################
bagname = sys.argv[1]
bag = rosbag.Bag('../bagfiles/' + bagname + '.bag') #Read bag

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

# Remove duplicate time instants and take last one of them as a true value
df_leader_poses.drop_duplicates(subset='t', keep = 'last', inplace = True)
df_leader_poses = df_leader_poses.reset_index(drop=True)

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
    
  
for robot_idx in range(1,total_num_of_robots):
    # Remove duplicate time instants and take last one of them as a true value
    boids_poses[robot_idx].drop_duplicates(subset='t', keep = 'last', inplace = True)
    boids_poses[robot_idx] = boids_poses[robot_idx].reset_index(drop=True)
    min_pose_robot_index = 0 # leader
    min_pose_msg_size = leader_pose_msg_size
    if boids_poses[robot_idx].shape[0] < min_pose_msg_size:
        min_pose_msg_size = boids_poses[robot_idx].shape[0]
        min_pose_robot_index = robot_idx

###### Data Check ############ 
#print("################ POSE DATA CHECK ############")
#print(df_leader_poses.shape)
#print(boids_poses[1].shape)
#print(boids_poses[2].shape)
#print(boids_poses[3].shape)
#print(boids_poses[4].shape)
#print(boids_poses[5].shape)
#print(boids_poses[6].shape)
#print(boids_poses[7].shape)
#print(boids_poses[8].shape)
#print(boids_poses[9].shape)
#print(boids_poses[10].shape)
#print(boids_poses[11].shape)
#print(boids_poses[12].shape)



## This part is to make sure that all obtained data is synchorized
for robot_idx in range(1,total_num_of_robots):
    if boids_poses[robot_idx].shape[0] > min_pose_msg_size:
        d = boids_poses[robot_idx].shape[0] - min_pose_msg_size #number of rows to delete
        for _ in range(d):
            if min_pose_robot_index == 0: #leader has the smallest size
                if boids_poses[robot_idx]['t'][0] != df_leader_poses['t'][0]:
                    # We are deleting first row
                    boids_poses[robot_idx] = boids_poses[robot_idx].iloc[1:,].reset_index(drop=True)
                else:
                    # We need to delete last row
                    boids_poses[robot_idx] = boids_poses[robot_idx][:-1]  
            else: # some agent other than leader has the smallest size
                if boids_poses[robot_idx]['t'][0] != boids_poses[min_pose_robot_index]['t'][0]:
                    # We are deleting first row
                    boids_poses[robot_idx] = boids_poses[robot_idx].iloc[1:,].reset_index(drop=True)
                else:
                    # We need to delete last row
                    boids_poses[robot_idx] = boids_poses[robot_idx][:-1]               

if min_pose_robot_index != 0: #leader doesn't have the smallest size
    d = df_leader_poses.shape[0] - min_pose_msg_size #number of rows to delete
    for _ in range(d):
        if df_leader_poses['t'][0] != boids_poses[min_pose_robot_index]['t'][0]:
            # We are deleting first row
            df_leader_poses = df_leader_poses.iloc[1:,].reset_index(drop=True)
        else:
            # We need to delete last row
            df_leader_poses = df_leader_poses[:-1]              


############## Read orientations
# Read leader orientation --> Since it is 2D, we need to subscribe cmd_vel
# arctan(cmd_vel.linear.y / cmd_vel.linear.x) will give the orientation, i.e. angle 
df_leader_angles = pd.DataFrame(columns=['angle','t'])
leader_counter = 0
for _ , msg, t in bag.read_messages("/robot_0/cmd_vel"):
        df_leader_angles.loc[leader_counter] = [np.degrees(np.arctan2(msg.linear.y,msg.linear.x))
                                                , t.to_sec()]
        leader_counter += 1

# Remove duplicate time instants and take last one of them as a true value
df_leader_angles.drop_duplicates(subset='t', keep = 'last', inplace = True)
df_leader_angles = df_leader_angles.reset_index(drop=True)

# Total number of orientation msgs leader published, this will be used to synchorize boids
leader_angle_msg_size = df_leader_angles.shape[0] 

boids_angles = {} # Dictionary for all boids angles through time 
# Example: boids[1] contains all angles for robot_1
# boids[total_num_of_robots] contains all angles for robot_total_num_of_robots 
for robot_idx in range(1,total_num_of_robots): #start from robot_1
    boids_angles[robot_idx] = pd.DataFrame(columns=['angle','t'])
    row_idx = 0
    for topic , msg, t in bag.read_messages("/robot_" + str(robot_idx) + "/cmd_vel"):
            boids_angles[robot_idx].loc[row_idx] = [np.degrees(np.arctan2(msg.linear.y,msg.linear.x))
                                                    , t.to_sec()]
            row_idx += 1
    
for robot_idx in range(1,total_num_of_robots):
    # Remove duplicate time instants and take last one of them as a true value
    boids_angles[robot_idx].drop_duplicates(subset='t', keep = 'last', inplace = True)
    boids_angles[robot_idx] = boids_angles[robot_idx].reset_index(drop=True)
    min_angle_robot_index = 0 #leader
    min_angle_msg_size = leader_angle_msg_size
    if boids_angles[robot_idx].shape[0] < min_angle_msg_size:
        min_angle_msg_size = boids_angles[robot_idx].shape[0]
        min_angle_robot_index = robot_idx

###### Data Check ############ 
#print("################ ANGLE DATA CHECK ############")
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


## This part is to make sure that all obtained data is synchorized
for robot_idx in range(1,total_num_of_robots):
    if boids_angles[robot_idx].shape[0] > min_angle_msg_size:
        d = boids_angles[robot_idx].shape[0] - min_angle_msg_size #number of rows to delete
        for _ in range(d):
            if min_angle_robot_index == 0: #leader has the smallest size
                if boids_angles[robot_idx]['t'][0] != df_leader_angles['t'][0]:
                    # We are deleting first row
                    boids_angles[robot_idx] = boids_angles[robot_idx].iloc[1:,].reset_index(drop=True)
                else:
                    # We need to delete last row
                    boids_angles[robot_idx] = boids_angles[robot_idx][:-1]  
            else: # some agent other than leader has the smallest size
                if boids_angles[robot_idx]['t'][0] != boids_angles[min_angle_robot_index]['t'][0]:
                    # We are deleting first row
                    boids_angles[robot_idx] = boids_angles[robot_idx].iloc[1:,].reset_index(drop=True)
                else:
                    # We need to delete last row
                    boids_angles[robot_idx] = boids_angles[robot_idx][:-1]               

if min_angle_robot_index != 0: #leader doesn't have the smallest size
    d = df_leader_angles.shape[0] - min_angle_msg_size #number of rows to delete
    for _ in range(d):
        if df_leader_angles['t'][0] != boids_angles[min_angle_robot_index]['t'][0]:
            # We are deleting first row
            df_leader_angles = df_leader_angles.iloc[1:,].reset_index(drop=True)
        else:
            # We need to delete last row
            df_leader_angles = df_leader_angles[:-1]            
        







#################### Calculate Metrics ######################################


##### Calculate relative distance to leader for each robot for all the time
boids_rel2leader_poses = {} # Dictionary for all boids distances to leader
# E.g. boids_rel2leader[2] will contain distance of robot_2 to leader for all the times
# boids_rel2leader[2] structure will be an pd dataframe with columns--> distance and t
for robot_idx in range(1,total_num_of_robots):
    boids_rel2leader_poses[robot_idx] = get_distance(df_leader_poses, boids_poses[robot_idx])
    print("#########Robot Poses %d" %(robot_idx))
    print(boids_rel2leader_poses[robot_idx])

##### Calculate relative angles to leader for each robot for all the time
boids_rel2leader_angles = {} # Dictionary for all boids angles relative to leader
# E.g. boids_rel2leader[2] will contain distance of robot_2 to leader for all the times
# boids_rel2leader[2] structure will be an pd dataframe with columns--> distance and t
for robot_idx in range(1,total_num_of_robots):
    boids_rel2leader_angles[robot_idx] = get_abs_difference(df_leader_angles,boids_angles[robot_idx]) 
    print("#########Robot Angles %d" %(robot_idx))
    print(boids_rel2leader_angles[robot_idx])

##### Separation Metric 
Q_sep_nominator = 0.0
for robot_idx in range(1,total_num_of_robots):
    t_sep = 0.0 #for each boid we are calculating separately
    total_sep_violation = 0 #Number of time instants one boid violates seperation
    seperation_check = boids_rel2leader_poses[robot_idx]['distance'] < separation_threshold
    total_sep_violation = boids_rel2leader_poses[robot_idx]['t'][seperation_check].shape[0]
    print("Seperation violation: %d" %(total_sep_violation))
    if total_sep_violation != 0:
        t_sep = total_sep_violation * 0.1 #There is 0.1 time difference between time instants
        Q_sep_nominator += t_sep

Q_sep = Q_sep_nominator / total_time #violation of seperation

##### Cohesion Metric 
Q_coh_nominator = 0.0
for robot_idx in range(1,total_num_of_robots):
    t_coh = 0.0 #for each boid we are calculating separately
    total_coh_violation = 0 #number of time instants one boid violates cohesion
    cohesion_check = boids_rel2leader_poses[robot_idx]['distance'] > cohesion_threshold
    total_coh_violation = boids_rel2leader_poses[robot_idx]['t'][cohesion_check].shape[0]
    print("Cohesion violation: %d" %(total_coh_violation))
    if total_coh_violation != 0:
        t_coh = total_coh_violation * 0.1 #There is 0.1 time difference between time instants
        Q_coh_nominator += t_coh

Q_coh = Q_coh_nominator / total_time #violation of seperation


##### Alignment Metric
Q_alig_nominator = 0.0
for robot_idx in range(1,total_num_of_robots):
    t_alig = 0.0 #for each boid we are calculating separately
    total_alig_violation = 0 #number of time instants one boid violates cohesion
    alignment_check = boids_rel2leader_angles[robot_idx]['angle'] > alignment_threshold
    total_alig_violation = boids_rel2leader_angles[robot_idx]['t'][alignment_check].shape[0]
    print("Alignment violation: %d" %(total_alig_violation))
    if total_alig_violation != 0:
        t_alig = total_alig_violation * 0.1 #There is 0.1 time difference between time instants
        Q_alig_nominator += t_alig

Q_alig = Q_alig_nominator / total_time #violation of seperation


print("Q_sep: %f" %(Q_sep))
print("Q_coh: %f" %(Q_coh))
print("Q_alig: %f" %(Q_alig))




#pd.set_option('display.max_rows', 1000)
bag.close()