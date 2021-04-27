# vui
Repository by Russell Valente
Code created for WPI RBE HRI
Worcester Polytechnic Institute (WPI)
Dept. of Robotics Engineering - Human-Robot Interaction Directed Research \

Download and install pyaudio.
Ubuntu:
$ sudo apt install python3-pyaudio
Windows:
A pip install for pyaudio may not be successful, the following website provides help to install:
https://stackoverflow.com/questions/61348555/error-pyaudio-0-2-11-cp38-cp38-win-amd64-whl-is-not-a-supported-wheel-on-this-p.
The helpful comment here points to a website to download pyaudio directly from here: https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio

    Install SpeechRecognition $ pip install SpeechRecognition


* Optional step: ensure ROS is installed (follow steps here: http://wiki.ros.org/noetic/Installation/Ubuntu).
* Create catkin directory
In terminal type:
$ mkdir -p catkin_ws/src
Copy github:
$ cd catkin_ws/src
$ git clone https://github.com/russcv/vui.git
If using Catkin make
$ source /opt/ros/noetic/setup.bash
$ cd ~/catkin_ws
$ catkin_make
Or if using catkin tools
$ source /opt/ros/noetic/setup.bash
$ cd ~/catkin_ws
$ catkin init
$ catkin build
If you are interesting in installing and using catkin tools instead: https://catkin-tools.readthedocs.io/en/latest/installing.html
You may need to install a couple dependencies (taken from https://answers.ros.org/question/355478/problems-with-catkin-with-ros-noetic/)
$ sudo apt install python3-catkin-lint python3-pip
$ pip3 install osrf-pycommon
Install dependencies for this workspace
Pip3 install SpeechRecognition
sudo apt install python3-pyaudio
Run the launch file:
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch workspace_gazebo workspace.launch
