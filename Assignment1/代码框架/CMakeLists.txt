cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer
    main.cpp
    rasterizer.hpp rasterizer.cpp
    Triangle.hpp Triangle.cpp
)

target_include_directories(Rasterizer
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(Rasterizer
    PRIVATE
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
)
