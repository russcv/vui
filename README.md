# vui
Repository by Russell Valente
Code created for WPI RBE HRI
Worcester Polytechnic Institute (WPI)
Dept. of Robotics Engineering - Human-Robot Interaction Directed Research \

1. Optional step: ensure ROS is installed (follow steps here: http://wiki.ros.org/noetic/Installation/Ubuntu).  
2. Create catkin directory  
In terminal type:  
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/russcv/vui.git
```

* If using catkin_make  
```
source /opt/ros/noetic/setup.bash  
cd ~/catkin_ws  
catkin_make  
```  
* Or if using catkin tools  
```
source /opt/ros/noetic/setup.bash  
cd ~/catkin_ws  
catkin init  
catkin build  
```  
* If you are interesting in installing and using catkin tools instead: https://catkin-tools.readthedocs.io/en/latest/installing.html
You may need to install a couple dependencies (taken from https://answers.ros.org/question/355478/problems-with-catkin-with-ros-noetic/)  
```
sudo apt install python3-catkin-lint python3-pip  
pip3 install osrf-pycommon
```  
  
3. Install dependencies for this workspace  
```
pip3 install SpeechRecognition  
sudo apt install python3-pyaudio
```
  
4. Run the launch file:
```
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch workspace_gazebo workspace.launch
```
