cmake_minimum_required(VERSION 3.10)

# set the project name
project(yorkapriltag)

#setup apriltag library
file(GLOB TAG_FILES src/tag*.c)
aux_source_directory(src/common COMMON_SRC)
set(APRILTAG_SRCS src/apriltag.c src/apriltag_pose.c src/apriltag_quad_thresh.c)
add_library(${PROJECT_NAME} ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})

# install header file hierarchy
file(GLOB HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/*.h src/common/*.h)
foreach(HEADER ${HEADER_FILES})
    string(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    install(FILES ${HEADER} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/${DIR})
endforeach()

find_package(catkin REQUIRED COMPONENTS
		      roscpp
		      pcl_conversions
		      pcl_ros
		      std_msgs
		      sensor_msgs
		      geometry_msgs
		      cv_bridge
		      image_transport)
		      
find_package(PCL 1.3 REQUIRED)
		      
find_package(Boost REQUIRED COMPONENTS 
		    thread 
		    system 
		    program_options)
		   
find_package(yaml-cpp REQUIRED)
		  
catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   pcl_conversions
                   pcl_ros
)

#export library
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(targets_export_name "${PROJECT_NAME}Targets")
set(config_install_dir "share/${PROJECT_NAME}/cmake")

# install library
install(TARGETS ${PROJECT_NAME} EXPORT ${targets_export_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

install(EXPORT ${targets_export_name}
    NAMESPACE apriltag::
    DESTINATION ${config_install_dir})

install(FILES ${project_config} ${version_config} DESTINATION ${config_install_dir})

export(TARGETS yorkapriltag
    NAMESPACE apriltag::
    FILE ${generated_dir}/${targets_export_name}.cmake)
    
#install resource directories
install(DIRECTORY resources
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories(/usr/local/include/yaml-cpp)		      
include_directories(${PCL_INCLUDE_DIRS}
		    ${catkin_INCLUDE_DIRS}
		    ${Boost_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS}
		  ${catkin_INCLUDE_DIRS} 
		  ${OpenCV_INCLUDE_DIRS})
		  
add_definitions(${PCL_DEFINITIONS})

# add the executable
add_executable(detect_apriltag src/main.cpp src/IILFM.cpp src/util.cpp)
target_link_libraries(detect_apriltag ${PCL_LIBRARIES}
					${catkin_LIBRARIES}
					${Boost_INCLUDE_DIRS}
					${OpenCV_LIBS}
					yorkapriltag
					yaml-cpp)
