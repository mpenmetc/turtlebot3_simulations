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

def write_dataset(stamp, x, y, z):
    with open(stamp+'_local.csv', 'wb') as f:
        writer = csv.writer(f, delimiter = ',')
        line_num = 0
        rows = []
        for (a, b, c) in zip(x, y, z):
            line_num+=1
            if line_num == 1: 
                rows.append([a[0],a[1],a[2], b[0],b[1],b[2],b[3], c[0],c[1],c[2]])
            else:
                rows.append([str(a[0]),str(a[1]),str(a[2]), str(b[0]),str(b[1]),str(b[2]),str(b[3]), str(c[0]),str(c[1]),str(c[2])])
        print(len(rows))
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

    """ bw_img_0 = rostopic.ROSTopicBandwidth()
    img_0 = rospy.Subscriber('/tb3_0camera/rgb/image_raw/compressed', rospy.AnyMsg, bw_img_0.callback)
    bw_img_1 = rostopic.ROSTopicBandwidth()
    img_1 = rospy.Subscriber('/tb3_1camera/rgb/image_raw/compressed', rospy.AnyMsg, bw_img_1.callback)
    bw_img_2 = rostopic.ROSTopicBandwidth()
    img_2 = rospy.Subscriber('/tb3_2camera/rgb/image_raw/compressed', rospy.AnyMsg, bw_img_2.callback) """

    """ bw_detectron_0 = rostopic.ROSTopicBandwidth()
    detectron_0 = rospy.Subscriber('/tb3_0/detectron2_ros/result', rospy.AnyMsg, bw_detectron_0.callback)
    bw_detectron_1 = rostopic.ROSTopicBandwidth()
    detectron_1 = rospy.Subscriber('/tb3_1/detectron2_ros/result', rospy.AnyMsg, bw_detectron_1.callback)
    bw_detectron_2 = rostopic.ROSTopicBandwidth()
    detectron_2 = rospy.Subscriber('/tb3_2/detectron2_ros/result', rospy.AnyMsg, bw_detectron_2.callback) """

    hz_map = rostopic.ROSTopicHz(-1)
    map_0 = rospy.Subscriber('/map', rospy.AnyMsg, hz_map.callback_hz, callback_args='/map')
    hz_map_0 = rostopic.ROSTopicHz(-1)
    map_0 = rospy.Subscriber('/tb3_0/map', rospy.AnyMsg, hz_map_0.callback_hz, callback_args='/tb3_0/map')
    hz_map_1 = rostopic.ROSTopicHz(-1)
    map_1 = rospy.Subscriber('/tb3_1/map', rospy.AnyMsg, hz_map_1.callback_hz, callback_args='/tb3_1/map')
    hz_map_2 = rostopic.ROSTopicHz(-1)
    map_2 = rospy.Subscriber('/tb3_2/map', rospy.AnyMsg, hz_map_2.callback_hz, callback_args='/tb3_2/map')

    """ hz_detectron_0 = rostopic.ROSTopicHz(-1)
    detectron_0 = rospy.Subscriber('/tb3_0/detectron2_ros/result', rospy.AnyMsg, hz_detectron_0.callback_hz)
    hz_detectron_1 = rostopic.ROSTopicHz(-1)
    detectron_1 = rospy.Subscriber('/tb3_1/detectron2_ros/result', rospy.AnyMsg, hz_detectron_1.callback_hz)
    hz_detectron_2 = rostopic.ROSTopicHz(-1)
    detectron_2 = rospy.Subscriber('/tb3_2/detectron2_ros/result', rospy.AnyMsg, hz_detectron_2.callback_hz) """


    rospy.loginfo("LOCAL >> Starting now executing the generated goals ....")
    plan_tb0.start()
    plan_tb1.start()
    plan_tb2.start()

    global resources_usage, bw_values, resources_pub, bw_pub
    scan_bw = []    # bandwidth = datasize/sec
    map_pt = []     # publishTime = 1/pubFreq
    resources_data = []

    resources_data.append(["RAM(%)", "CPU(%)", "GPU(%"])
    scan_bw.append(["laser_0_bw(B/sec)", "laser_1_bw(B/sec)", "laser_2_bw(B/sec)"])
    map_pt.append(["map_0_execT(sec)", "map_1_execT(sec)", "map_2_execT(sec)", "fullMap_execT(sec)"])

    rospy.loginfo("LOCAL >> Starting storing the dataset in python lists ....")
    try:
        while not rospy.is_shutdown():
            global scan_bw, map_pt 
            try:
                #resources_usage.header.frame_id = platform
                #resources_usage.header.stamp = rospy.Time.now()

                cpu_percent = ps.cpu_percent()
                #resources_usage.cpu = cpu_percent
                ram_percent = ps.virtual_memory()[2]
                #resources_usage.ram = ram_percent
                gpu_percent = get_gpu_memory()
                #resources_usage.gpu = gpu_percent
                
                resources_data.append([round(ram_percent, 3), round(cpu_percent, 3), round(gpu_percent, 3)])
                #resources_pub.publish(resources_usage)
            
                #bw_values.header.frame_id = platform
                #bw_values.header.stamp = rospy.Time.now()
                
                scan_0 = float(bw_scan_0.get_bw())            
                #bw_values.scan_0 = scan_0
                scan_1 = float(bw_scan_1.get_bw())
                #bw_values.scan_1 = scan_1
                scan_2 = float(bw_scan_2.get_bw())
                #bw_values.scan_2 = scan_2
                
                scan_bw.append([scan_0, scan_1, scan_2])

                full_map = hz_map.get_hz("/map")[0]
                #hz_values.map = full_map
                map_1 = hz_map_1.get_hz("/tb3_1/map")[0]
                #hz_values.map_1 = map_1
                map_2 = hz_map_2.get_hz("/tb3_2/map")[0]
                #hz_values.map_2 = map_2
                map_0 = hz_map_0.get_hz("/tb3_0/map")[0]
                #hz_values.map_0 = map_0
                map_pt.append([round(1.0/map_0, 3), round(1.0/map_1, 3), round(1.0/map_2, 3), round(1.0/full_map, 3)])


                """ bw_values.detectron_0 = bw_detectron_0.get_bw()
                bw_values.detectron_1 = bw_detectron_1.get_bw()
                bw_values.detectron_2 = bw_detectron_2.get_bw() """
                
                #bw_pub.publish(bw_values)
            except:
                pass
                # print(type(bw_scan_0.get_bw()))
                # print(hz_map_0.get_hz("/tb3_0/map"))
                # hz_map_0.print_hz(["/tb3_0/map"])
                # print("################################")
    except KeyboardInterrupt:
        rospy.loginfo("LOCAL >> Navigation compeleted!")   

    rospy.loginfo("LOCAL >> Starting writing the dataset in csv file ....")
    write_dataset(str(rospy.Time.now().to_nsec()), scan_bw, map_pt, resources_data)     
    rospy.loginfo("LOCAL >> Now dataset is ready ....")
    rospy.spin()

if __name__ == '__main__':
    main()