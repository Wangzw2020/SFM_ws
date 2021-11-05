cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11)
project(social_force_model)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(


)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

find_package(glfw3 REQUIRED)

add_executable(opengl_show src/opengl_show.cpp)
target_link_libraries(opengl_show ${catkin_LIBRARIES})
target_link_libraries(opengl_show ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} glfw)

add_executable(ros_time_run src/ros_time_run.cpp)

