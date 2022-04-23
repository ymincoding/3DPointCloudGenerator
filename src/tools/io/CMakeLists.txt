#==============================================================================
# SS22 Project Test Task: io
#==============================================================================

add_library(io SHARED "" PlyFileWriter.cpp PlyFileWriter.h)

#------------------------------------------------------------------------------
# Configure target
#------------------------------------------------------------------------------
target_sources(io
  PUBLIC
    FileIO.h
    FileWriter.h
    imageIO.h
    PlyFileWriter.h
  PRIVATE
    FileIO.cpp
    FileIO.hpp
    FileWriter.cpp
    FileWriter.hpp
    imageIO.cpp
    PlyFileWriter.cpp
  )

target_include_directories(io SYSTEM
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )

link_directories(
    ${OpenCV_LIB_DIR}
  )

set_property(TARGET io PROPERTY
    OUTPUT_NAME "IO"
  )

target_link_libraries(io
  INTERFACE
    glog::glog
    opencv_core
    opencv_imgcodecs
  )

#------------------------------------------------------------------------------
# Install rules
#------------------------------------------------------------------------------
install(TARGETS io ${DEFAULT_TARGET_INSTALL_DESTINATIONS})