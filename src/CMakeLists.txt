cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(HERACLES)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(../../Dep/liblas/include)
include_directories(../headers)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(HERACLES main.cpp
			las2pcd.cpp
			PARVIS.cpp
			ROSACE.cpp
			PIER.cpp)

target_link_libraries(HERACLES 
${PCL_LIBRARIES}
../../Dep/liblas/lib/liblas
../../Dep/liblas/lib/liblas_c
)
