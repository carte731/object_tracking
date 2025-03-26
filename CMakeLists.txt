cmake_minimum_required(VERSION 2.6.4)
project(CFE_OBJECT_TRACKER C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)

# Sync-Node library functions
include_directories(${sync_node_lib_MISSION_DIR}/fsw/public_inc)
include_directories(${sync_node_lib_MISSION_DIR}/fsw/backend)

aux_source_directory(fsw/src APP_SRC_FILES)

# Create the app module
add_cfe_app(object_tracker ${APP_SRC_FILES})
