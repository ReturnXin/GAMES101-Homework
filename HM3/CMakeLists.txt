cmake_minimum_required(VERSION 3.10)
project(HM3)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("C:\\Program Files (x86)\\Eigen3\\include")

add_executable(HM3 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(HM3 ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)