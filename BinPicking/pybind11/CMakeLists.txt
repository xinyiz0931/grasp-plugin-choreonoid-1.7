set(target PyBinPicking)

add_cnoid_python_module(${target}
	PyBinPickingModule.cpp
	PyMotionPlanner.cpp
)
target_link_libraries(${target} CnoidGraspPlugin CnoidPyBase PyBody PyBodyPlugin)
