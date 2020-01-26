cmake_minimum_required(VERSION 2.8.3)
project(abb_librws_samples)

find_package(catkin REQUIRED COMPONENTS roscpp abb_librws)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp abb_librws
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#################################
# Build the basic level samples #
#################################

add_executable(${PROJECT_NAME}_collect_info_node src/a1_collect_info_node.cpp)
set_target_properties(${PROJECT_NAME}_collect_info_node PROPERTIES OUTPUT_NAME a1_collect_info_node PREFIX "")
add_dependencies(${PROJECT_NAME}_collect_info_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_collect_info_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_io_signal_node src/a2_io_signal_node.cpp)
set_target_properties(${PROJECT_NAME}_io_signal_node PROPERTIES OUTPUT_NAME a2_io_signal_node PREFIX "")
add_dependencies(${PROJECT_NAME}_io_signal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_io_signal_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_rapid_execution_node src/a3_rapid_execution_node.cpp)
set_target_properties(${PROJECT_NAME}_rapid_execution_node PROPERTIES OUTPUT_NAME a3_rapid_execution_node PREFIX "")
add_dependencies(${PROJECT_NAME}_rapid_execution_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rapid_execution_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_file_service_node src/a4_file_service_node.cpp)
set_target_properties(${PROJECT_NAME}_file_service_node PROPERTIES OUTPUT_NAME a4_file_service_node PREFIX "")
add_dependencies(${PROJECT_NAME}_file_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_file_service_node ${catkin_LIBRARIES})

########################################
# Build the intermediate level samples #
########################################

add_executable(${PROJECT_NAME}_rapid_data_node src/b1_rapid_data_node.cpp)
set_target_properties(${PROJECT_NAME}_rapid_data_node PROPERTIES OUTPUT_NAME b1_rapid_data_node PREFIX "")
add_dependencies(${PROJECT_NAME}_rapid_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rapid_data_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_subscription_node src/b2_subscription_node.cpp)
set_target_properties(${PROJECT_NAME}_subscription_node PROPERTIES OUTPUT_NAME b2_subscription_node PREFIX "")
add_dependencies(${PROJECT_NAME}_subscription_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_subscription_node ${catkin_LIBRARIES})

####################################
# Build the advanced level samples #
####################################

add_executable(${PROJECT_NAME}_state_machine_interface_node src/c1_state_machine_interface_node.cpp)
set_target_properties(${PROJECT_NAME}_state_machine_interface_node PROPERTIES OUTPUT_NAME c1_state_machine_interface_node PREFIX "")
add_dependencies(${PROJECT_NAME}_state_machine_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_state_machine_interface_node ${catkin_LIBRARIES})
