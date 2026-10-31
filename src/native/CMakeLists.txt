cmake_minimum_required(VERSION 3.5)
project(pidrone_ekf CXX)

add_library(pidrone_ekf SHARED ekf.cpp)
set_target_properties(pidrone_ekf PROPERTIES
  COMPILE_FLAGS "-O3 -std=c++11"
  SUFFIX ".so")
install(TARGETS pidrone_ekf LIBRARY DESTINATION lib)

if(catkin_FOUND)
  find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc imgcodecs video calib3d)
  find_package(Threads REQUIRED)
  add_library(pidrone_ekf_live STATIC live_ekf.cpp)
  target_include_directories(pidrone_ekf_live PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pidrone_ekf_live pidrone_ekf ${OpenCV_LIBRARIES})
  set_target_properties(pidrone_ekf_live PROPERTIES COMPILE_FLAGS "-O3 -std=c++11")
  add_executable(ekf_localization_node ekf_node.cpp)
  target_include_directories(ekf_localization_node PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ekf_localization_node pidrone_ekf_live ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  add_dependencies(ekf_localization_node pidrone_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  set_target_properties(ekf_localization_node PROPERTIES COMPILE_FLAGS "-O3 -std=c++11")
  install(TARGETS ekf_localization_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
