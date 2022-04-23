#==============================================================================
# SS22 Project Test Task: app
#==============================================================================

add_executable(ss22_project_test
  main.cpp
  )

target_include_directories(ss22_project_test SYSTEM
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )

link_directories(
    ${OpenCV_LIB_DIR}
  )

set_property(TARGET ss22_project_test PROPERTY
    OUTPUT_NAME "ss22ProjectTest"
  )

target_link_libraries(ss22_project_test
  PRIVATE
    cmd_line_parser
    core
    io
    utils
  )