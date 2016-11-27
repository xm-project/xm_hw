##xm_hw node
###Description: A ros node to communicate between ROS and STM32F4
###Usage
1. install [roscontrol](http://wiki.ros.org/ros_control) packages that required.
> sudo apt-get install ros-indigo-control*
2. Change the path in main.cpp to your computer path.
> xm_hw::xm_hw_ros xm(nh, "serial:///dev/ttyUSB0", "/home/username/your_path/xm_hw/config/config.txt");
3.build the package 
> catkin_make
4. if you want to change the speed of communicate with the STM32F4 change the config.txt in xm_hw/config/config.txt
5. roslaunch xm_hw xm_hw.launch
