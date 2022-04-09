add_executable(${PROJECT_NAME}_test so_test.cpp)
target_link_libraries(${PROJECT_NAME}_test PUBLIC ${PROJECT_NAME})
