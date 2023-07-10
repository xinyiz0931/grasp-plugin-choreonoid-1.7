set(target PyPRM)

add_cnoid_python_module(${target}
	PyPRMModule.cpp
	PyTrajectoryPlanner.cpp


)
target_link_libraries(${target} CnoidGraspPlugin CnoidPyBase PyBody PyBodyPlugin)
