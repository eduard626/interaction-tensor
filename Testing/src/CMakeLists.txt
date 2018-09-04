cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Voronoi)

find_package(PCL 1.8 REQUIRED)
find_package(OpenMP)

# From the PCL src files. Just copied since
# it was easier to solve cuda flags and stuff
include(find_cuda.cmake)


if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#link_directories(${CUDA_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

message(${PCL_INCLUDE_DIRS})


add_executable (meshSampling mesh_sampling.cpp)
target_link_libraries (meshSampling ${PCL_LIBRARIES})

add_executable (convert_pcd_ascii_binary convert_pcd_ascii_binary.cpp)
target_link_libraries (convert_pcd_ascii_binary ${PCL_LIBRARIES})

# VTK and CUDA do not get along 
# so manually remove _vtk_definitions from path at compilation
# as shown in PCL examples
get_directory_property(_dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(_vtk_definitions)
foreach(_item ${_dir_defs})
	if(_item MATCHES "vtk*")
		list(APPEND _vtk_definitions -D${_item})
	endif()
endforeach()
remove_definitions(${_vtk_definitions})

CUDA_ADD_LIBRARY(myCuda
    SHARED
    myCuda.cu
)
TARGET_LINK_LIBRARIES(myCuda
    ${CUDA_LIBRARIES}
)

CUDA_ADD_LIBRARY(dev_array
    SHARED
    dev_array.h
)
TARGET_LINK_LIBRARIES(dev_array
    ${CUDA_LIBRARIES}
)

add_executable (tensorTest Tensor.cpp main.cpp)
target_link_libraries (tensorTest myCuda dev_array ${PCL_LIBRARIES})
set_target_properties(tensorTest PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
