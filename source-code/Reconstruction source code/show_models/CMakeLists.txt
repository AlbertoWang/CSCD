cmake_minimum_required(VERSION 3.12)
project(ShowModel)

set(CMAKE_CXX_STANDARD 14)

# for write ply 
# include_directories(vendors/happly)


# for basic rendering
add_subdirectory(polyscope)


set(BIN_NAME show_models)

add_executable(${BIN_NAME} 
	./show_models.cpp 
	./draw_models_impl.cpp 
	)

target_link_libraries(${BIN_NAME} 
	polyscope
	#${ASSIMP_LIBRARIES}
	# ${pcl_component}
	)
