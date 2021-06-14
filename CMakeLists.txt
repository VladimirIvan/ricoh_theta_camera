cmake_minimum_required(VERSION 3.1.0)
project(ricoh_theta_camera)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libav REQUIRED libavcodec libavutil libavformat libswscale)
pkg_check_modules(libuvc REQUIRED libuvc)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${libuvc_INCLUDE_DIRS}
  ${libjpeg_INCLUDE_DIRS}
  ${libav_INCLUDE_DIRS}
)

add_executable(ricoh_theta_camera src/theta_ros.cpp src/thetauvc.c)
target_link_libraries(ricoh_theta_camera ${libuvc_LIBRARIES} ${catkin_LIBRARIES} Threads::Threads ${libuvc_LDFLAGS} ${libav_LIBRARIES})

install(TARGETS ricoh_theta_camera
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)