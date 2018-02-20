cmake_minimum_required(VERSION 2.8.8)
project(evol)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
	cmake_modules
  dvs_msgs
)

# since we need boost search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)


# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS 
  	include
    ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    dvs_msgs
  DEPENDS
    OpenCV
    Eigen
  LIBRARIES
    evol
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


file(GLOB TEST_SRC_FILES
  ${PROJECT_SOURCE_DIR}/test/*.cc)




file(GLOB_RECURSE HEADERS include/event_tracker/*.hpp include/event_tracker/*.h)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


add_executable(evol_node  ${PROJECT_SOURCE_DIR}/src/main.cc)
target_link_libraries (evol_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
