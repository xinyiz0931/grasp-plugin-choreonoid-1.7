cmake_minimum_required(VERSION 2.6)

#find_package(Lapack REQUIRED)

if(WIN32)
set(LAPACKE_TOP_DIR ${LAPACKE_TOP_DIR} CACHE PATH "The directory of the LPACKE library")
set(LAPACK_LIBRARY_DIRS ${LAPACKE_TOP_DIR}/lib)
set(LAPACK_DLL_DIRS ${LAPACKE_TOP_DIR}/Release)
endif()

include_directories(${LAPACK_INCLUDE_DIRS})
link_directories(${LAPACK_LIBRARY_DIRS})

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(right_shared
  ikfast61.HRP2STEP1_RARM.${CMAKE_SYSTEM_PROCESSOR} 
  )
  
set(left_shared
  ikfast61.HRP2STEP1_LARM.${CMAKE_SYSTEM_PROCESSOR} 
  )

set(target 
  right_shared left_shared
  )

set(right_source
  ikfast61.HRP2STEP1.Transform6D_right.cpp
  )

set(left_source
  ikfast61.HRP2STEP1.Transform6D_left.cpp
  )

add_definitions(-DIKFAST_NO_MAIN -DIKFAST_CLIBRARY)

add_library(${left_shared} SHARED ${left_source})
set_target_properties(${left_shared} PROPERTIES PREFIX "")
target_link_libraries(${left_shared} ${LAPACK_LIBRARIES})
if(WIN32)
target_link_libraries(${left_shared} liblapack)
else()
target_link_libraries(${left_shared} lapack)
endif()

add_library(${right_shared} SHARED ${right_source})
set_target_properties(${right_shared} PROPERTIES PREFIX "")
target_link_libraries(${right_shared} ${LAPACK_LIBRARIES})
if(WIN32)
target_link_libraries(${right_shared} liblapack)
else()
target_link_libraries(${right_shared} lapack)
endif()

if(MSVC)
install(TARGETS ${left_shared} ${right_shared}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
)
foreach(dll liblapack.dll libblas.dll libgcc_s_dw2-1.dll libgfortran-3.dll)
  install(FILES "${LAPACK_DLL_DIRS}/${dll}" DESTINATION bin)
endforeach()
else()
install(TARGETS ${left_shared} ${right_shared}
  LIBRARY DESTINATION ${ROBOT}/lib CONFIGURATIONS Release Debug
)
endif()
