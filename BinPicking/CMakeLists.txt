set(target CnoidBinPickingPlugin)

# set(sources
#   BinPickingBar.cpp)
# set(headers
#   BinPickingBar.h)

# if(USE_QT5)
#   find_package(Qt5PrintSupport)
#   include_directories(${Qt5PrintSupport_INCLUDE_DIRS})
# endif()

# add_cnoid_plugin(${target} SHARED ${sources} ${headers})

# add_definitions(-g)

# set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

# target_link_libraries(${target} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin)

# apply_common_setting_for_plugin(${target})

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
