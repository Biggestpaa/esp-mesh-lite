cmake_minimum_required(VERSION 3.5)

# IMPORTANT: include repo-level components (mesh_lite, wifi_provisioning, etc.)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(no_router)
