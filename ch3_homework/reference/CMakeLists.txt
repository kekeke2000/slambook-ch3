cmake_minimum_required(VERSION 2.8)

project(ch3_homework)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

find_package(Eigen3)

include_directories(${EIGEN3_INCLUDE_DIRS})

# add_executable(answer ch3_reference.cpp)

# target_link_libraries(answer ${EIGEN3_LIBRARIES})

# add_executable(zhuanhuan zhuanhuan.cpp)

# target_link_libraries(zhuanhuan ${EIGEN3_LIBRARIES})
add_executable(zhuanhuan3 zhuanhuan3.cpp)

target_link_libraries(zhuanhuan3 ${EIGEN3_LIBRARIES})

add_executable(zhuanhuan4 zhuanhuan4.cpp)

target_link_libraries(zhuanhuan4 ${EIGEN3_LIBRARIES})

# add_executable(zhuanhuan4 ceshi.cpp)

# target_link_libraries(zhuanhuan4 ${EIGEN3_LIBRARIES})
# add_executable(zhuanhuan5 zhuanhuan5.cpp)

# target_link_libraries(zhuanhuan5 ${EIGEN3_LIBRARIES})
