#!/usr/bin/env python
import rospy

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
        rospy.loginfo("Navigation finished.")

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

def main():
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('tasks_scheduler')
    points = gen_nSamples_points(int(sys.argv[1]))
    allocate_points(points)
    plan_tb0 = threading.Thread(target=exec_robot_points, args=("/tb3_0/", points_0))
    plan_tb1 = threading.Thread(target=exec_robot_points, args=("/tb3_1/", points_1))
    plan_tb2 = threading.Thread(target=exec_robot_points, args=("/tb3_2/", points_2))
    plan_tb0.start()
    plan_tb1.start()
    plan_tb2.start()
    rospy.spin()
if __name__ == '__main__':
    main()