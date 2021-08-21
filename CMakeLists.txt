cmake_minimum_required(VERSION 3.8)
project(Deform2D)

find_package(OpenGL)
find_package(OpenMP)

set(CMAKE_CXX_STANDARD 14)

if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OpenMP_FOUND)

file(GLOB_RECURSE Deform2D_sources Deform2D/*.cpp)
file(GLOB_RECURSE Deform2D_headers Deform2D/*.h)
add_library(Deform2D SHARED ${Deform2D_sources})
target_include_directories(Deform2D PRIVATE third/eigen)

if(OpenGL_FOUND)
    option(FREEGLUT_BUILD_DEMOS OFF)
    add_subdirectory(third/freeglut-3.2.1)

    file(GLOB_RECURSE App_sources App/*.cpp)
    file(GLOB_RECURSE App_headers App/*.h)
    add_executable(App ${App_sources})
    target_include_directories(App PRIVATE Deform2D third/eigen third/freeglut-3.2.1/include)
    target_link_libraries(App Deform2D ${OPENGL_LIBRARY} freeglut_static)
endif(OpenGL_FOUND)

