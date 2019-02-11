# SR_robot

Main package of KTB Prototype Service robot. Include Robot's URDF, launch files, ROS Mapping & Navigation parameter, and firmware for STM32 NucleoF411RE.
<img src="https://user-images.githubusercontent.com/21339780/52552936-8f4e2e80-2e14-11e9-8bec-175584288dcc.JPG" alt="main"/>

## System Requirement
- Ubuntu 16.04LTS
- ROS Kinetic Kame
### Update by using git pull
On local machine
```
cd ~/catkin_ws/src/srobot/
```
```
git pull
```
### Installing package by catkin_make
In case of reinstall. Please install on Intel NUC7 miniPC on the robot.

Install this as a ROS package by using catkin_make.

1. Clone this Rep into catkin_ws.
```
cd ~/catkin_ws/src/
```
```
git clone -b master https://github.com/poorinxD/SR_robot.git
```
2. Install dependancy using rosdep.
```
cd .. && rosdep install srobot
```
3. Make by catkin_make on catkin_ws directory.
```
catkin_make
```

## Running SLAM & Navigation
### Running Navigation GUI app
GUI app only works with default map.
1. Make sure that all the hardwares are powered running Lidar, Wheel Motor's Battery,Nucleo F411RE, OpenCR.
2. Navigation in KTB Innovationlab with GUI app can be run via desktop icon "Robot Navigation".
![screenshot from 2018-12-03 15-21-29](https://user-images.githubusercontent.com/21339780/52558612-6f733680-2e25-11e9-98f7-5cddcc9dc5eb.png)
3. The GUI app will show with Rviz. Rviz can be close if not needed.
![gui](https://user-images.githubusercontent.com/21339780/52558988-9aaa5580-2e26-11e9-88da-40c793860318.png)
4. If the robot did not start in the correct starting location. Use 2D Pose Estimate tool in Rviz to mark robot's current location within the map.
5. Assign goal by pressing any location button on the GUI app, then press "GO" to start navigation.
6. Robot will stop when goal is reached or press stop to stop navigation.
7. In case of malfunction press the emergency button to stop the motor. Then press "Exit program" and restart the program. 

### Running Navigation without GUI app
1. Make sure that all the hardwares are powered running Lidar, Wheel Motor's Battery,Nucleo F411RE, OpenCR.
2. Launch navigation operation with default map file.
```
roslaunch srobot navigation.launch
```
or with specific map file
```
roslaunch srobot navigation.launch map_file:=[filepath.yaml]
```
For more Action API please refer to [ROS Move_base](http://wiki.ros.org/move_base).
 
### Running SLAM Mapping
In case of making new map other than default location (KTB Innovation lab).
1. Make sure that all the hardwares are powered running Lidar, Wheel Motor's Battery,Nucleo F411RE, OpenCR.
2. Launch slam operation
```
roslaunch srobot slam.launch
```
3. Launch keyboard teleop
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
![screenshot from 2018-12-17 15-44-08](https://user-images.githubusercontent.com/21339780/52558067-ddb6f980-2e23-11e9-80f2-4472346a8a42.png)
4. Manually control the robot by staying on teleop_key tab and press wasdx. Controll at low speed 0-3 m/s.

5. Save the map by running map_saver node.
```
rosrun map_server map_saver -f ~/map
```


## Built With

* [ROS](http://www.ros.org/) - Robot operating system framework


## Authors

* **Phurin Rangpong** - email: poorin31632@gmail.com, [Facebook](https://www.facebook.com/poorin.rangpong)




