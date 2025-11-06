cmake_minimum_required(VERSION 3.22)

project(camera_thingy)

SET(camera_sources "main.cpp")
add_executable(camera_demo ${camera_sources})
add_executable(camera_depth_picker "depth_picker.cpp")

find_package(SDL2 CONFIG REQUIRED)
set_property(TARGET camera_demo         PROPERTY CXX_STANDARD 20)
set_property(TARGET camera_depth_picker PROPERTY CXX_STANDARD 20)

if(WIN32)
    find_package(realsense2 CONFIG REQUIRED)
    set(CAMERA_DEPENDENCIES realsense2::realsense2 realsense2::rsutils realsense2::fw realsense2::realsense-file)
    target_link_libraries(camera_demo           PRIVATE ${CAMERA_DEPENDENCIES} SDL2::SDL2 SDL2::SDL2main)
    target_link_libraries(camera_depth_picker   PRIVATE ${CAMERA_DEPENDENCIES} SDL2::SDL2 SDL2::SDL2main)
else()
    find_package(PNG REQUIRED)
    add_subdirectory(librealsense-2.56.5)
    set(CAMERA_DEPENDENCIES realsense2 tclap)
    target_link_libraries(camera_demo           PRIVATE ${CAMERA_DEPENDENCIES} SDL2::SDL2 SDL2::SDL2main)
    target_link_libraries(camera_depth_picker   PRIVATE ${CAMERA_DEPENDENCIES} SDL2::SDL2 SDL2::SDL2main)
endif()
