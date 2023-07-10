# @ author Xinyi Zhang
set (target PyMotionFile)

add_cnoid_python_module(
    ${target}
    PyMotionFileModule.cpp
    PyMotionFileControl.cpp
)
target_link_libraries(${target} CnoidGraspPlugin CnoidPybase PyBody PyBodyPlugin)