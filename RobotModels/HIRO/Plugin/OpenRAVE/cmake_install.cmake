# Install script for directory: /home/hlab/choreonoid-1.7.0/ext/graspPlugin/RobotModels/HIRO/Plugin/OpenRAVE

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Dd][Ee][Bb][Uu][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so"
           RPATH "$ORIGIN/..")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/lib/ikfast61.HIRONX_LARM.x86_64.so")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/lib" TYPE SHARED_LIBRARY FILES "/home/hlab/choreonoid-1.7.0/ext/graspPlugin/RobotModels/HIRO/Plugin/ikfast61.HIRONX_LARM.x86_64.so")
    if(EXISTS "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so"
           OLD_RPATH "/home/hlab/choreonoid-1.7.0/lib:/home/hlab/choreonoid-1.7.0/lib/choreonoid-1.7:"
           NEW_RPATH "$ORIGIN/..")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/lib/ikfast61.HIRONX_LARM.x86_64.so")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Dd][Ee][Bb][Uu][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so"
           RPATH "$ORIGIN/..")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/lib/ikfast61.HIRONX_RARM.x86_64.so")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/lib" TYPE SHARED_LIBRARY FILES "/home/hlab/choreonoid-1.7.0/ext/graspPlugin/RobotModels/HIRO/Plugin/ikfast61.HIRONX_RARM.x86_64.so")
    if(EXISTS "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so"
           OLD_RPATH "/home/hlab/choreonoid-1.7.0/lib:/home/hlab/choreonoid-1.7.0/lib/choreonoid-1.7:"
           NEW_RPATH "$ORIGIN/..")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/lib/ikfast61.HIRONX_RARM.x86_64.so")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Dd][Ee][Bb][Uu][Gg])$")
endif()

