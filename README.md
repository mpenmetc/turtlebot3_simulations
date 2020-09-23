# AWS Offloading for Multi-Turtlebot3 Gazebo Simulation

![image](https://user-images.githubusercontent.com/29764281/93975015-ffee5880-fd76-11ea-89a0-7606930ab8d1.png)


## 1. Prerequisites
### 1.1 **Ubuntu 18.04** and **ROS**

### 1.2. **Install ROS Nav Stack**
```
sudo apt install ros-melodic-navigation ros-melodic-map-server ros-melodic-move-base 
```
### 1.3. **Install Detectron2 Object Detector for ROS**
It is necessary to install Detectron2 [requirements](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md) in a python *virtual environment* as it requires `Python 3.6` and ROS works with `Python 2.7`

1. Install python Virtual Environment
```bash
sudo apt-get install python-pip
sudo pip install virtualenv
mkdir ~/.virtualenvs
sudo pip install virtualenvwrapper
export WORKON_HOME=~/.virtualenvs
echo '. /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc 
```

2. Creating Virtual Environment
```bash
mkvirtualenv --python=python3 detectron2_ros
```

3. [Install the dependencies in the virtual environment](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md)

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
### Start the multi-turtlebot3 Gmapping SLAM

```bash
roslaunch turtlebot3_gazebo exec_multi_slam.launch
```
### Start the multi-turtlebot3 Navigation Stack

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
