# Install script for directory: /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/amir/Desktop/ardrone_simulator/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/msg" TYPE FILE FILES
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/Altimeter.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/Altitude.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/AttitudeCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/Compass.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/ControllerState.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/HeadingCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/HeightCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/MotorCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/MotorPWM.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/MotorStatus.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/PositionXYCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/RawImu.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/RawMagnetic.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/RawRC.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/RC.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/RuddersCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/ServoCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/Supply.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/ThrustCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/VelocityXYCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/VelocityZCommand.msg"
    "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/msg/YawrateCommand.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES "/home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/amir/Desktop/ardrone_simulator/devel/include/cvg_sim_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/amir/Desktop/ardrone_simulator/devel/share/common-lisp/ros/cvg_sim_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/amir/Desktop/ardrone_simulator/devel/lib/python2.7/dist-packages/cvg_sim_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/amir/Desktop/ardrone_simulator/devel/lib/python2.7/dist-packages/cvg_sim_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES "/home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES
    "/home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgsConfig.cmake"
    "/home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs" TYPE FILE FILES "/home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_msgs/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

