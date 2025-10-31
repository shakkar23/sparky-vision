cmake_minimum_required(VERSION 3.22)

project(camera_thingy)

SET(camera_sources "main.cpp")
add_executable(camera ${camera_sources})

find_package(SDL2 CONFIG REQUIRED)

if(WIN32)
    set_property(TARGET camera PROPERTY CXX_STANDARD 20)
    find_package(realsense2 CONFIG REQUIRED)
    set(CAMERA_DEPENDENCIES realsense2::realsense2 realsense2::rsutils realsense2::fw realsense2::realsense-file)
    target_link_libraries(camera PRIVATE ${CAMERA_DEPENDENCIES} SDL2::SDL2 SDL2::SDL2main)
else()
    find_package(PNG REQUIRED)
    set_property(TARGET camera PROPERTY CXX_STANDARD 20)
    add_subdirectory(librealsense-2.56.5)
    set(CAMERA_DEPENDENCIES realsense2 tclap)
    target_link_libraries(camera PRIVATE ${CAMERA_DEPENDENCIES} PNG::PNG SDL2::SDL2 SDL2::SDL2main)
endif()
