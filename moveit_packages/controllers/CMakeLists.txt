include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(position_control src/position_control.cpp)
target_link_libraries(position_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_scene src/planning_scene.cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_scene2 src/planning_scene2.cpp)
target_link_libraries(planning_scene2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(roll_motion_kinova src/roll_motion_kinova.cpp)
target_link_libraries(roll_motion_kinova ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(roll_motion 
	src/roll_motion.cpp
	src/utils.cpp )
target_link_libraries(roll_motion ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(full_mission src/full_mission.cpp)
target_link_libraries(full_mission ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(position_control_with_callback src/position_control_with_callback.cpp)
target_link_libraries(position_control_with_callback ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(pose_control src/pose_control.cpp)
target_link_libraries(pose_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cartesian_control src/cartesian_control.cpp)
target_link_libraries(cartesian_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Link to the marker package to be able to access the custom message
find_package(marker1)
catkin_package(CATKIN_DEPENDS message_runtime)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


