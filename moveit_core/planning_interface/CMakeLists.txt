set(MOVEIT_LIB_NAME moveit_planning_interface)

add_library(${MOVEIT_LIB_NAME}
  src/planning_response.cpp
  src/planning_interface.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
