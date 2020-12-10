# AWS Offloading for Multi-Turtlebot3 Gazebo Simulation

![image](https://user-images.githubusercontent.com/29764281/93975015-ffee5880-fd76-11ea-89a0-7606930ab8d1.png)

## 1. Prerequisites
### 1.1 **Ubuntu 18.04** and **ROS**

### 1.2. **Install ROS Nav Stack**
```
sudo apt install ros-melodic-navigation ros-melodic-map-server ros-melodic-move-base 
```
### 1.3. AWS-integration Setup
1.3.2. Install OpenVPN on your machine
```bash
sudo apt-get install openvpn 
```
1.3.2. Download openvpn client configuration file to your home directory from [here](https://drive.google.com/file/d/1LFRQ60WWJLMDFvbzzCmjaEsa4nByWWje/view?usp=sharing) 
1.3.3. Download ssh private key file to your ~/.ssh directory from [here](https://drive.google.com/file/d/13PwXrtuiGrag2FifiP4wqtv-eJthIVvT/view?usp=sharing) 

### 1.4. **Install Detectron2 Object Detector for ROS**
It is necessary to install Detectron2 [requirements](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md) in a python *virtual environment* as it requires `Python 3.6` and ROS works with `Python 2.7`

1.4.1. Install python Virtual Environment
```bash
sudo apt-get install python-pip
sudo pip install virtualenv
mkdir ~/.virtualenvs
sudo pip install virtualenvwrapper
export WORKON_HOME=~/.virtualenvs
echo '. /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc 
```

1.4.2. Creating Virtual Environment
```bash
mkvirtualenv --python=python3 detectron2_ros
```

1.4.3. [Install the dependencies in the virtual environment](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md)

```bash
pip install -U torch==1.4+cu100 torchvision==0.5+cu100 -f https://download.pytorch.org/whl/torch_stable.html
pip install cython pyyaml==5.1
pip install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu100/index.html
pip install opencv-python
pip install rospkg
```

## 2. Build & Install
Clone the repository (offloading-dev branch), install dependencies, and build the workspace:

```
cd ~/catkin_ws/src
rm -rf *
git clone https://github.com/mhaboali/turtlebot3_simulations.git
cd turtlebot3_simulations
git checkout offloading-dev
cd ../
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
source ~/catkin_ws/devel/setup.bash
```

## 3. Usage Instructions
You have to open 5 different terminals and write one command per each terminal to get the full system running
### Start the multi-turtlebot3 Gazebo simulation
```bash
roslaunch turtlebot3_gazebo multi_turtlebot3_aws_warehouse.launch
```
#### Start the multi-turtlebot3 Gmapping SLAM
```bash
roslaunch turtlebot3_gazebo exec_multi_slam.launch
```
#### Start the multi-turtlebot3 Navigation Stack
```bash
roslaunch turtlebot3_gazebo exec_multi_nav.launch
```
### Start the multi-turtlebot3 detectron2 
Open a new terminal and use the virtual environment created.
```bash
workon detectron2_ros
roslaunch turtlebot3_gazebo exec_multi_detectron2.launch
```
### Start the multi-turtlebot3 tasks allocator and dataset-writer node
```bash
mkdir ~/catkin_ws/dataset
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/dataset
export NUM_GOALS_PER_ROBOT=10
rosrun turtlebot3_gazebo eval.py $NUM_GOALS_PER_ROBOT

And press Ctrl + c when you want to write the dataset CSV file
```
All the previous steps explained how to run the entire system completely on the local machine without any AWS integration. You can follow the following steps to get your AWS EC2 properly integrated.

## AWS-offloaded system instructions
### 1. Start the instance from the EC2 dashboard
### 2. Connect to the instance from your local machine through 4 terminals using this command
```bash
ssh -i "~/.ssh/Awstest.pem" openvpnas@ec2-3-22-184-43.us-east-2.compute.amazonaws.com
```
### Source ROS on aws - melodic
### 3. Run these commands initially in each of them
```bash
export ROS_MASTER_URI=http://172.31.4.86:11311
export ROS_HOSTNAME=172.31.4.86
export ROS_IP=172.31.4.86
```
### 4. Run the openvpn client on your local machine using this command
```bash
sudo openvpn ~/client-g4dn_8xlarge.ovpn
```
**User: ** openvpn
**Password: ** test
### 5. Run the ROS master in one of those AWS terminals using `roscore`
### 6. On the local machine we need 2 terminals configured with the AWS by running the following commands:
```bash
export ROS_MASTER_URI=http://172.31.4.86:11311
export ROS_HOSTNAME=172.27.232.2
export ROS_IP=172.27.232.2
```
### 7. Get the system running
#### On Local machine
The first command you should run is this command
##### Start the multi-turtlebot3 Gazebo simulation
```bash
roslaunch turtlebot3_gazebo multi_turtlebot3_aws_warehouse.launch
```
##### Start the multi-turtlebot3 tasks allocator and dataset-writer node
The last command you should run after running all other launch files on local and AWS machine.
```bash
mkdir ~/catkin_ws/dataset
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/dataset
export NUM_GOALS_PER_ROBOT=10
rosrun turtlebot3_gazebo eval.py $NUM_GOALS_PER_ROBOT

And press Ctrl + c when you want to write the dataset CSV file
```
#### On AWS EC2 machine
##### Start the multi-turtlebot3 Gmapping SLAM
```bash
roslaunch turtlebot3_gazebo exec_multi_slam.launch
```
##### Start the multi-turtlebot3 Navigation Stack
```bash
roslaunch turtlebot3_gazebo exec_multi_nav.launch
```
#### Start the multi-turtlebot3 detectron2 
Open a new terminal and use the virtual environment created.
```bash
workon detectron2_ros
roslaunch turtlebot3_gazebo exec_multi_detectron2.launch
```
