cmake_minimum_required(VERSION 3.10)

project(arm)
set(CMAKE_CXX_STANDARD 14)


find_package(OpenCV REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm CONFIG REQUIRED)

target_link_libraries(main PRIVATE glm::glm)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(arm/include)
add_executable(arm main.cpp get_path.cpp calculations.cpp render.cpp glad.c)
target_link_libraries(arm ${OpenCV_LIBS})
target_link_libraries(arm PRIVATE glfw OpenGL::GL)