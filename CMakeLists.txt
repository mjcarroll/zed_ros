cmake_minimum_required(VERSION 2.8.3)
project(zed_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs cv_bridge dynamic_reconfigure image_transport nodelet camera_info_manager)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  SHARED
  src/nodelet/zed_ros.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(zed_driver_node
  src/node/driver.cpp
)

target_link_libraries(
  zed_driver_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
       DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS zed_driver_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
