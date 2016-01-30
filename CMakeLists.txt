cmake_minimum_required(VERSION 2.8.3)
project(qrcode_detection)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  nodelet
  tf
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

add_message_files(
  FILES
  QRCode.msg
  QRCodeArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/QRCodeDetection.cpp
  src/qrcode_detection_nodelet.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/qrcode_detection_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

