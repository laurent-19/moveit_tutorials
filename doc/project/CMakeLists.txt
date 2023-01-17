add_executable(pick_dowork_place_action src/pick_dowork_place_action.cpp)
target_link_libraries(pick_dowork_place_action ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_dowork_place_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})