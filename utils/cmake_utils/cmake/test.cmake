macro (CMAKE_ADD_TEST NAME)
   add_executable(${NAME}_test test/${NAME}_test.cpp)
   target_link_libraries(${NAME}_test gtest pthread ${PROJECT_NAME})
   if("${DRMEMORY}" STREQUAL "")
       add_test(NAME ${NAME}_test
                COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
   else()
       add_test(NAME ${NAME}_test
                COMMAND ${DRMEMORY} -brief -results_to_stderr -exit_code_if_errors 2 -- ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
   endif()
endmacro (CMAKE_ADD_TEST)

macro (CMAKE_ADD_NODE NAME)
   add_executable(${NAME}_node node/${NAME}_node.cpp)
   add_dependencies(${NAME}_node ${catkin_EXPORTED_TARGETS})
   target_link_libraries(${NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${${NAME}__LIBRARIES})
endmacro (CMAKE_ADD_NODE NAME)

