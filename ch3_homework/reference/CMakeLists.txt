cmake_minimum_required(VERSION 2.8)

project(ch3_homework)

find_package(Eigen3)

include_directories(${EIGEN3_INCLUDE_DIRS})

# add_executable(answer ch3_reference.cpp)

# target_link_libraries(answer ${EIGEN3_LIBRARIES})

# add_executable(zhuanhuan zhuanhuan.cpp)

# target_link_libraries(zhuanhuan ${EIGEN3_LIBRARIES})
add_executable(zhuanhuan2 zhuanhuan2.cpp)

target_link_libraries(zhuanhuan2 ${EIGEN3_LIBRARIES})