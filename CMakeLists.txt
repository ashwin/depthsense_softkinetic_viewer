cmake_minimum_required(VERSION 2.8)

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/bin
    )

include_directories(
    /opt/softkinetic/DepthSenseSDK/include
    )

link_directories(
    /opt/softkinetic/DepthSenseSDK/lib
    )

link_libraries(
    -lDepthSense
    -lopencv_core
    -lopencv_highgui
    -lopencv_imgproc
    -lopencv_contrib
    )

add_executable(
    depthsense_softkinetic_viewer
    src/ConsoleDemo.cxx
    )
