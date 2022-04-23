#==============================================================================
# SS22 Project Test Task: core
#==============================================================================

add_library(core SHARED "")

#------------------------------------------------------------------------------
# Configure target
#------------------------------------------------------------------------------
target_sources(core
  PUBLIC
    Constants.h
    EigenTypes.h
    MathUtils.h
  PRIVATE
    MathUtils.cpp
    MathUtils.hpp
  )

target_include_directories(core SYSTEM
  PUBLIC
    ${EIGEN3_INCLUDE_DIR}
  )

set_property(TARGET core PROPERTY
  OUTPUT_NAME "Core"
  )

#------------------------------------------------------------------------------
# Install rules
#------------------------------------------------------------------------------
install(TARGETS core ${DEFAULT_TARGET_INSTALL_DESTINATIONS})
