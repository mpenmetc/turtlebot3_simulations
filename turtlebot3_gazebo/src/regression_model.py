#!/usr/bin/env python
import rospy
import pandas as pd  
import numpy as np  
import matplotlib.pyplot as plt  
import seaborn as seabornInstance 
from sklearn.model_selection import train_test_split 
from sklearn.linear_model import LinearRegression
from sklearn import metrics

platform = "local"

def read_dataset(path):
    dataset = pd.read_csv(path)
    return dataset

def main():
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('regression_model')
    ds_path_dir = ""
    if platform == "local":
        ds_path_dir = "/media/h/d888433a-f54c-4a8d-ae71-d14563ae93bc/offloading_ws/dataset/csv/"
    else:
        ds_path_dir = "/home/openvpnas/catkin_ws/dataset/csv/"
    
    dataset = read_dataset(ds_path_dir+"128332000000.csv")
    print(dataset.shape)

    rospy.loginfo("Starting storing the dataset in python lists ....")
    try:
        while not rospy.is_shutdown():
            pass        
    except KeyboardInterrupt:
        rospy.loginfo("Program Interrupted!")
    rospy.spin()

if __name__ == '__main__':
    main()