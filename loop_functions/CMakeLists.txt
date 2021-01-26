link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_tracking)
set(loop_functions_SOURCES loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(loop_functions_SOURCES
    ${loop_functions_SOURCES}
    foraging_qt_user_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(loop_functions MODULE ${loop_functions_SOURCES})
target_link_libraries(loop_functions
  footbot_tracking
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)
