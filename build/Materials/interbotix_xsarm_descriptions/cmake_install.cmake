# Install script for directory: /home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/gvnk/ros1_ur5_arm_project_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/gvnk/ros1_ur5_arm_project_ws/build/Materials/interbotix_xsarm_descriptions/catkin_generated/installspace/interbotix_xsarm_descriptions.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_xsarm_descriptions/cmake" TYPE FILE FILES
    "/home/gvnk/ros1_ur5_arm_project_ws/build/Materials/interbotix_xsarm_descriptions/catkin_generated/installspace/interbotix_xsarm_descriptionsConfig.cmake"
    "/home/gvnk/ros1_ur5_arm_project_ws/build/Materials/interbotix_xsarm_descriptions/catkin_generated/installspace/interbotix_xsarm_descriptionsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_xsarm_descriptions" TYPE FILE FILES "/home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_xsarm_descriptions" TYPE DIRECTORY FILES
    "/home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions/launch"
    "/home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions/meshes"
    "/home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions/rviz"
    "/home/gvnk/ros1_ur5_arm_project_ws/src/Materials/interbotix_xsarm_descriptions/urdf"
    )
endif()

