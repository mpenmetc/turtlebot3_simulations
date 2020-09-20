#!/usr/bin/env python
import rospy
import rostopic
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

from numpy.random import seed
from numpy.random import randint
import random
import threading
import sys
import psutil as ps
import subprocess as sp
import os
from turtlebot3_msgs.msg import ResourcesList, BwList

import csv
from itertools import izip

platform = "local"

goals_counter = 0

resources_pub = rospy.Publisher(platform+'/resources_usage', ResourcesList, queue_size=10)

bw_pub = rospy.Publisher(platform+'/bw', BwList, queue_size=10)

resources_usage = ResourcesList()
bw_values = BwList()

def get_gpu_memory():
  _output_to_list = lambda x: x.decode('ascii').split('\n')[:-1]

  ACCEPTABLE_AVAILABLE_MEMORY = 1024
  COMMAND = "nvidia-smi --query-gpu=memory.free --format=csv"
  memory_free_info = _output_to_list(sp.check_output(COMMAND.split()))[1:]
  memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
  return (11175.0 - memory_free_values[0])/11175.0



# seed random number generator
seed(1)

points_0 = []
points_1 = []
points_2 = []

def movebase_client_cmd(ns, pos):
    global goals_counter
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient(ns+'move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = pos[0]
    goal.target_pose.pose.position.y = pos[1]
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
    goals_counter += 1
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def exec_cmd(ns,pos):
    try:
        result = movebase_client_cmd(ns, pos)
        if result:
            rospy.loginfo(ns + ":       Goal execution done!")
            return result
    
    except rospy.ROSInterruptException:
        rospy.loginfo("LOCAL >> Navigation finished.")

def exec_robot_points(ns, points):
    for pnt in points:
        res = exec_cmd(ns, pnt)        

def gen_x(min, max):
    return randint(min, max, 3)

def gen_y(min, max):
   return random.sample(xrange(min, max), 3)

def gen_oneSample_points():
    mode = randint(0,1,1)
    points = []
    x_list = []
    y_list = []
    if mode == 0:
        x_list = gen_x(0,2)
        y_list = gen_y(-9,9)
    else:
        x_list = gen_x(-5,2)
        y_list = gen_y(-9,0)
    return zip(x_list,y_list)

def gen_nSamples_points(n):
    points = []
    for i in range(0,n):
        points.append(gen_oneSample_points())
    print(points)
    return points

def allocate_points(points):
    for sample in points:
        points_0.append(sample[0])
        points_1.append(sample[1])
        points_2.append(sample[2])
        
def write_dataset(stamp, x, y, y_bw, z):
    with open(stamp+'_local.csv', 'wb') as f:
        writer = csv.writer(f, delimiter = ',')
        line_num = 0
        rows = []
        for (a, b, b_bw, c) in zip(x, y, y_bw, z):
            line_num+=1
            if line_num == 1: 
                rows.append([a[0],a[1],a[2], b[0],b[1],b[2],b[3], b_bw[0],b_bw[1],b_bw[2],b_bw[3], c[0],c[1],c[2]])
            else:
                rows.append([str(a[0]),str(a[1]),str(a[2]), str(b[0]),str(b[1]),str(b[2]),str(b[3]), str(b_bw[0]),str(b_bw[1]),str(b_bw[2]),str(b_bw[3]), str(c[0]),str(c[1]),str(c[2])])
        print(rows)
        writer.writerows(rows)
            # writer.writerows([x.split(',') for x in row_str])
def main():
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('local_evaluator')
    points = gen_nSamples_points(int(sys.argv[1]))
    allocate_points(points)
    plan_tb0 = threading.Thread(target=exec_robot_points, args=("/tb3_0/", points_0))
    plan_tb1 = threading.Thread(target=exec_robot_points, args=("/tb3_1/", points_1))
    plan_tb2 = threading.Thread(target=exec_robot_points, args=("/tb3_2/", points_2))

    bw_scan_0 = rostopic.ROSTopicBandwidth()
    scan_0 = rospy.Subscriber('/tb3_0/scan', rospy.AnyMsg, bw_scan_0.callback)
    bw_scan_1 = rostopic.ROSTopicBandwidth()
    scan_1 = rospy.Subscriber('/tb3_1/scan', rospy.AnyMsg, bw_scan_1.callback)
    bw_scan_2 = rostopic.ROSTopicBandwidth()
    scan_2 = rospy.Subscriber('/tb3_2/scan', rospy.AnyMsg, bw_scan_2.callback) 

    bw_map = rostopic.ROSTopicBandwidth()
    full_map = rospy.Subscriber('/map', rospy.AnyMsg, bw_map.callback)
    bw_map_0 = rostopic.ROSTopicBandwidth()
    map_0 = rospy.Subscriber('/tb3_0/map', rospy.AnyMsg, bw_map_0.callback)
    bw_map_1 = rostopic.ROSTopicBandwidth()
    map_1 = rospy.Subscriber('/tb3_1/map', rospy.AnyMsg, bw_map_1.callback)
    bw_map_2 = rostopic.ROSTopicBandwidth()
    map_2 = rospy.Subscriber('/tb3_2/map', rospy.AnyMsg, bw_map_2.callback)
    
    hz_map = rostopic.ROSTopicHz(-1)
    hzmap_ = rospy.Subscriber('/map', rospy.AnyMsg, hz_map.callback_hz, callback_args='/map')
    hz_map_0 = rostopic.ROSTopicHz(-1)
    hzmap_0 = rospy.Subscriber('/tb3_0/map', rospy.AnyMsg, hz_map_0.callback_hz, callback_args='/tb3_0/map')
    hz_map_1 = rostopic.ROSTopicHz(-1)
    hzmap_1 = rospy.Subscriber('/tb3_1/map', rospy.AnyMsg, hz_map_1.callback_hz, callback_args='/tb3_1/map')
    hz_map_2 = rostopic.ROSTopicHz(-1)
    hzmap_2 = rospy.Subscriber('/tb3_2/map', rospy.AnyMsg, hz_map_2.callback_hz, callback_args='/tb3_2/map')

    rospy.loginfo("LOCAL >> Starting now executing the generated goals ....")
    plan_tb0.start()
    plan_tb1.start()
    plan_tb2.start()

    global resources_usage, bw_values, resources_pub, bw_pub
    scan_bw = []    # bandwidth = datasize/sec
    map_bw = []     # publishTime = 1/pubFreq
    map_pt = []
    resources_data = []

    resources_data.append([platform + "_RAM(%)", platform + "_CPU(%)", platform + "_GPU(%)"])
    scan_bw.append([platform + "_laser_0_bw(KB/sec)", platform + "_laser_1_bw(KB/sec)", platform + "_laser_2_bw(KB/sec)"])
    map_bw.append([platform + "_map_0_bw(KB/sec)", platform + "_map_1_bw(KB/sec)", platform + "_map_2_bw(KB/sec)", platform + "_fullMap_bw(KB/sec)"])
    map_pt.append([platform + "_map_0_execT(sec)", platform + "_map_1_execT(sec)", platform + "_map_2_execT(sec)", platform + "_fullMap_execT(sec)"])

    rospy.loginfo("LOCAL >> Starting storing the dataset in python lists ....")
    try:
        while not rospy.is_shutdown():            
            try:
                cpu_percent = ps.cpu_percent(interval=0.1)                
                ram_percent = ps.virtual_memory()[2]
                gpu_percent = get_gpu_memory()
                
                resources_data.append([round(ram_percent, 3), round(cpu_percent, 3), round(gpu_percent, 3)])
               
                scan_0 = float(bw_scan_0.get_bw() / 1000.0)            
                scan_1 = float(bw_scan_1.get_bw() / 1000.0)
                scan_2 = float(bw_scan_2.get_bw() / 1000.0)
                
                scan_bw.append([scan_0, scan_1, scan_2])

                full_map = float(bw_map.get_bw() / 1000.0)
                map_0 = float(bw_map_0.get_bw() / 1000.0)            
                map_1 = float(bw_map_1.get_bw() / 1000.0)
                map_2 = float(bw_map_2.get_bw() / 1000.0)
                map_bw.append([round(map_0, 3), round(map_1, 3), round(map_2, 3), round(full_map, 3)])

                full_map = hz_map.get_hz("/map")[0]                
                map_1 = hz_map_1.get_hz("/tb3_1/map")[0]                
                map_2 = hz_map_2.get_hz("/tb3_2/map")[0]                
                map_0 = hz_map_0.get_hz("/tb3_0/map")[0]               
                map_pt.append([round(1.0/map_0, 3), round(1.0/map_1, 3), round(1.0/map_2, 3), round(1.0/full_map, 3)])
                print("## Goals_Counter: %i", goals_counter)
            except:
                # print(map_pt)
                # print("################################")
                pass                
    except KeyboardInterrupt:
        rospy.loginfo("LOCAL >> Navigation compeleted!")   

    rospy.loginfo("LOCAL >> Starting writing the dataset in csv file ....")
    write_dataset(str(rospy.Time.now().to_nsec()), scan_bw, map_pt, map_bw, resources_data)     
    rospy.loginfo("LOCAL >> Now dataset is ready ....")
    rospy.spin()

if __name__ == '__main__':
    main()