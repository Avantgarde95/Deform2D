cmake_minimum_required(VERSION 3.8)
project(Deform2D)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenGL REQUIRED)

option(FREEGLUT_BUILD_DEMOS OFF)
add_subdirectory(third/freeglut-3.2.1)

add_subdirectory(third/Wml)

file(GLOB_RECURSE Deform2D_sources Deform2D/*.cpp)
file(GLOB_RECURSE Deform2D_headers Deform2D/*.h)
add_library(Deform2D SHARED ${Deform2D_sources})
target_include_directories(Deform2D PRIVATE third/Wml/Include)
target_link_libraries(Deform2D WildMagic)

file(GLOB_RECURSE App_sources App/*.cpp)
file(GLOB_RECURSE App_headers App/*.h)
add_executable(App ${App_sources})
target_include_directories(App PRIVATE Deform2D third/Wml/Include third/freeglut-3.2.1/include)
target_link_libraries(App WildMagic Deform2D ${OPENGL_LIBRARY} freeglut_static)
