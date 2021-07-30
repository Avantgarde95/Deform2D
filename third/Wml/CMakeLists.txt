cmake_minimum_required(VERSION 3.8)
project(WildMagic)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE WildMagic_headers Source/*.h Source/*.inl Source/*.mcr)
file(GLOB_RECURSE WildMagic_sources Source/*.cpp)

add_library(WildMagic STATIC ${WildMagic_sources})

foreach(header IN LISTS WildMagic_headers)
	message(STATUS "Copying ${header}...")
	file(COPY ${header} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/Include)
endforeach()

target_include_directories(WildMagic PRIVATE Include)
