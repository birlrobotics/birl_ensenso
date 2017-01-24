cmake_minimum_required(VERSION 2.8.3)
project(ensenso)

#add_definitions(-std=c++11)
#catkin_python_setup()
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  std_msgs
  std_srvs
  tf_conversions
  image_transport
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL 1.7.1 EXACT)
find_package(OpenCV 2 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/ensenso/cmake")
find_package(Ensenso REQUIRED)

add_service_files(
  FILES
  CapturePattern.srv
  ComputeCalibration.srv
  ConfigureStreaming.srv
  GridSpacing.srv
  InitCalibration.srv
  Lights.srv
  SetBool.srv
  CaptureSinglePointCloud.srv
  RegistImage.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
    cfg/CameraParameters.cfg
)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ensenso_grabber
  CATKIN_DEPENDS 
    camera_info_manager
    cv_bridge
    eigen_conversions
    message_runtime
    geometry_msgs
    std_msgs
    std_srvs
    pcl_conversions
    pcl_ros
    tf_conversions
)

# Order of the include dirs matters!
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ENSENSO_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Libraries
add_library(ensenso_grabber src/ensenso_grabber.cpp)
target_link_libraries(ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${ENSENSO_LIBRARIES} ${OpenCV_LIBRARIES})

#add_library(ensenso_ros_driver src/ensenso_ros_driver.cpp)
#target_link_libraries(ensenso_ros_driver ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ensenso_node src/ensenso_node.cpp)
add_dependencies(ensenso_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ensenso_node ensenso_grabber  ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ensenso_driver_node src/ensenso_driver_node.cpp)
add_dependencies(ensenso_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(ensenso_driver_node ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#simple client for test
add_executable(utility_client script/utility_client.cpp)
add_dependencies(ensenso_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(utility_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#implement segmentation
#add_executable(pointcloud_segmentation script/segmentation.cpp)
#target_link_libraries(pointcloud_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(region_growing_watershed script/region_growing_watershed.cpp)
#target_link_libraries(region_growing_watershed ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#depth and color registration test
add_executable(depth_color_regis script/depth_color_registration.cpp)
target_link_libraries(depth_color_regis  ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
