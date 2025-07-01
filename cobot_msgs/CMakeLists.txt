PROJECT(cobot_msgs)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

if(${ROS_VERSION} EQUAL "1")
  message(STATUS "Compiling with ROS1 ROSBUILD")
  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  ROSBUILD_INIT()
  SET(ROS_BUILD_STATIC_LIBS true)
  SET(ROS_BUILD_SHARED_LIBS false)

  # Set the directory for message and service files
  FILE(GLOB_RECURSE MSG_FILES msg/*.msg)
  FILE(GLOB_RECURSE SRV_FILES srv/*.srv)

  ROSBUILD_GENMSG()
  ROSBUILD_GENSRV()
elseif(${ROS_VERSION} EQUAL "2")
  message(STATUS "Compiling with ROS2 COLCON")
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AccelerationLimits.msg"
    "msg/ArduinoSensorMsg.msg"
    "msg/CobotAccelerometerMsg.msg"
    "msg/CobotAccessPointsMsg.msg"
    "msg/CobotAndroidUsbMsg.msg"
    "msg/CobotAnomalyMonitorMsg.msg"
    "msg/CobotAnomalyMonitorRectangleMsg.msg"
    "msg/CobotAskQuestionMsg.msg"
    "msg/CobotCameraMsg.msg"
    "msg/CobotDockQRMsg.msg"
    "msg/CobotDoorDetectionsMsg.msg"
    "msg/CobotDriveMsg.msg"
    "msg/CobotDriveRawMsg.msg"
    "msg/CobotEventsMsg.msg"
    "msg/CobotExecutionFaultMsg.msg"
    "msg/CobotExecutionMonitoringMsg.msg"
    "msg/CobotFloorNumber.msg"
    "msg/CobotFloorNumberMsg.msg"
    "msg/CobotFollowPathMsg.msg"
    "msg/CobotHumanClassified.msg"
    "msg/CobotHumanDetected.msg"
    "msg/CobotHumansClassified.msg"
    "msg/CobotHumansDetected.msg"
    "msg/CobotInteractionGUIMsg.msg"
    "msg/CobotInteractionStatusMsg.msg"
    "msg/CobotJoystickMsg.msg"
    "msg/CobotKinectStatusMsg.msg"
    "msg/CobotLocalization3DMsg.msg"
    "msg/CobotLocalizationMsg.msg"
    "msg/CobotLocalizationMsgRev1854.msg"
    "msg/CobotLocalizationMsgRev1881.msg"
    "msg/CobotLocalizationMsgRev1899.msg"
    "msg/CobotLocalizationMsgRev2043.msg"
    "msg/CobotLocalizationMsgRev378.msg"
    "msg/CobotLocationAndStarGazerMsg.msg"
    "msg/CobotModeMsg.msg"
    "msg/CobotMotionPrimitiveMsg.msg"
    "msg/CobotMotionStatusMsg.msg"
    "msg/CobotOdometryMsg.msg"
    "msg/CobotOdometryMsgRev337.msg"
    "msg/CobotParameterMsg.msg"
    "msg/CobotPathMsg.msg"
    "msg/CobotPersonArrayMsg.msg"
    "msg/CobotPersonMsg.msg"
    "msg/CobotRemoteMonitoring.msg"
    "msg/CobotRollingMapInfoMsg.msg"
    "msg/CobotSpeechRecognitionMsg.msg"
    "msg/CobotStarGazerAbsoluteMsg.msg"
    "msg/CobotStarGazerMarkerMsg.msg"
    "msg/CobotStarGazerMarkerStatusMsg.msg"
    "msg/CobotStarGazerMsg.msg"
    "msg/CobotStarGazerStatusMsg.msg"
    "msg/CobotStarGazerVelMsg.msg"
    "msg/CobotStatusMsg.msg"
    "msg/CobotStatusMsgRev1818.msg"
    "msg/CobotStatusMsgRev2247.msg"
    "msg/CobotStatusMsgRev779.msg"
    "msg/CobotTalkerMsg.msg"
    "msg/CobotTaskMsg.msg"
    "msg/CobotTaskPlannerMsg.msg"
    "msg/CobotUserResponseMsg.msg"
    "msg/CobotWallDetectionsMsg.msg"
    "msg/CobotWifiControlMsg.msg"
    "msg/CobotWifiMsg.msg"
    "msg/CobotWifiSamplerMsg.msg"
    "msg/CobotWifiSnifferMsg.msg"
    "msg/CobotWifiSpeedtestMsg.msg"
    "msg/CobotWifiStatusMsg.msg"
    "msg/DistanceSensorMsg.msg"
    "msg/GuiKeyboardEvent.msg"
    "msg/GuiMouseClickEvent.msg"
    "msg/GuiMouseMoveEvent.msg"
    "msg/LidarDisplayMsg.msg"
    "msg/LocalizationMsg.msg"
    "msg/MotionModel2D.msg"
    "msg/Trajectory2D.msg"
    DEPENDENCIES sensor_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
  )

  # --- Add install steps ---
  # If you have headers in an include directory, install them:
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    install(
      DIRECTORY include/
      DESTINATION include
    )
  endif()

  # Install the generated libraries
  install(
    DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp/" 
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so"
  )


  # Install the generated interface files and any message, srv, or action files.
  install(
    DIRECTORY msg srv action
    DESTINATION share/${PROJECT_NAME}
  )

  # Export runtime dependencies
  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
else()
  message(FATAL_ERROR "Unknown ROS_VERSION ${ROS_VERSION}")
endif()