cmake_minimum_required(VERSION 2.8.3)

project(srobot)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  roslaunch
  std_msgs
  move_base_msgs 
  actionlib  
)

catkin_package()



foreach(dir config launch meshes urdf nodes param rviz rqt desktop)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#add_executable(srobot src/goalsender_node.cpp)
#target_link_libraries(srobot ${catkin_LIBRARIES})
