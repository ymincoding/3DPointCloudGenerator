#==============================================================================
# SS22 Project Test Task: command line parser
#==============================================================================

add_library(cmd_line_parser SHARED "")

#------------------------------------------------------------------------------
# Configure target
#------------------------------------------------------------------------------
target_sources(cmd_line_parser
  PUBLIC
    CommandLine.h
  PRIVATE
    CommandLine.cpp
  )

set_property(TARGET cmd_line_parser PROPERTY
  OUTPUT_NAME "CommandLineParser"
  )

target_link_libraries(cmd_line_parser
  INTERFACE
    glog::glog
  )

#------------------------------------------------------------------------------
# Install rules
#------------------------------------------------------------------------------
install(TARGETS cmd_line_parser ${DEFAULT_TARGET_INSTALL_DESTINATIONS})
