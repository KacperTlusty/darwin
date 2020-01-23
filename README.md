# Darwin Gazebo

## Setup

Install ROS using instructions from [ROS official install guide](http://wiki.ros.org/melodic/Installation).

Init catkin workspace 
```bash
mkdir <your directory>
cd <your directory>
mkdir src
cd src
catkin_init_workspace
```

Clone repository

```bash
git clone https://github.com/HumaRobotics/darwin_description.git
git clone https://github.com/HumaRobotics/darwin_control.git
git clone https://github.com/KacperTlusty/darwin.git
cd ..
```

Build catkin project

```bash
catkin_make
```

Start `roscore` if it is not already running:
```bash
roscore &
```

Source from local setup file while in repository directory (replace **.bash** with **.zsh** if you're using ZSH):
```bash
catkin_make
source devel/setup.bash
```

Start gazebo with darwin robot
```bash
roslaunch darwin_gazebo darwin_gazebo.launch
```

Open new terminal window, source again and start script:
```bash
cd <your directory>
source devel/setup.bash
rosrun darwin_control fall.py
```

## Credits

This repository is bases on repositories created by [HumaRobotics](https://github.com/HumaRobotics):

* https://github.com/HumaRobotics/darwin_control
* https://github.com/HumaRobotics/darwin_gazebo
* https://github.com/HumaRobotics/darwin_description
