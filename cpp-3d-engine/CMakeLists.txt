cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Test VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)

link_directories(${Eigen_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_subdirectory(external/eigen-3.4.0)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
)
set(CMAKE_CXX_FLAGS "-O0")

