cmake_minimum_required(VERSION 3.10)

project(arm)
set(CMAKE_CXX_STANDARD 17)


find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm CONFIG REQUIRED)

add_executable(arm src/main.cpp src/get_path.cpp src/calculations.cpp src/render.cpp src/glad.c )
include_directories(
	include 
	${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(arm PRIVATE glm::glm ${OpenCV_LIBS} glfw OpenGL::GL Eigen3::Eigen)