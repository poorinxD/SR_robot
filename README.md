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
3. Make by catkin_make on catkin_ws directory
```
catkin_make
```

## Running the tests


### Break down into end to end tests


## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used


## Authors

* **Phurin Rangpong** - email: poorin31632@gmail.com




