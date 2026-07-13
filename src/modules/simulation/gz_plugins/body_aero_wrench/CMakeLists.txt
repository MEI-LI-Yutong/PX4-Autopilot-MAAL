############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

project(BodyAeroWrenchPlugin)

add_library(${PROJECT_NAME} SHARED
    BodyAeroWrenchSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
