# CMake version.
cmake_minimum_required(VERSION 3.8)

# Project name.
project(Deform2D)

# Dependencies.
find_package(OpenGL)
find_package(OpenMP)

# C++ version.
set(CMAKE_CXX_STANDARD 14)

# Compiler & linker flags.
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OpenMP_FOUND)

# MSVC-specific compiler & linker flags.
if(MSVC)
	add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
endif(MSVC)

# Build a library. ("Deform2D")
set(Deform2D_sources Deform2D/Deform2D.cpp Deform2D/RigidMeshDeformer2D.cpp)
set(Deform2D_headers Deform2D/Common.h Deform2D/Deform2D.h Deform2D/RigidMeshDeformer2D.h)
add_library(Deform2D SHARED ${Deform2D_sources} ${Deform2D_headers})
target_include_directories(Deform2D PRIVATE third/eigen)

if(OpenGL_FOUND)
    # Build some dependencies.
    option(FREEGLUT_BUILD_DEMOS OFF)
    add_subdirectory(third/freeglut-3.2.1)

    # Build an executable. ("App")
    set(App_sources App/LinearAlgebra.cpp App/TriangleMesh.cpp App/App.cpp)
    set(App_headers App/LinearAlgebra.h App/TriangleMesh.h)
    add_executable(App ${App_sources} ${App_headers})
    target_include_directories(App PRIVATE Deform2D third/eigen third/freeglut-3.2.1/include)
    target_link_libraries(App Deform2D ${OPENGL_LIBRARY} freeglut_static)
endif(OpenGL_FOUND)
