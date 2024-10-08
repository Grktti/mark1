cmake_minimum_required(VERSION 3.26)
project(multi_agent_systems)

set(CMAKE_CXX_STANDARD 20)


find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
message(STATUS "OPENGL_INCLUDE_DIR=${OPENGL_INCLUDE_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_executable(multi_agent_systems main.cpp crlAgentCore.hpp crlAgentCore_config.h
        crlAgent.hpp)


if (WIN32)
    if (MSVC)
        set(MSVC_COMPILE_FLAGS "/MP /utf-8 /Zc:__cplusplus")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_COMPILE_FLAGS}" )
        target_link_directories(multi_agent_systems PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/MSVC/)
        target_link_libraries(multi_agent_systems PRIVATE ${OPENGL_LIBRARIES} glfw3)
        target_compile_definitions(multi_agent_systems PRIVATE _USE_MATH_DEFINES) # for M_PI
    else ()
        target_link_directories(multi_agent_systems PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/WIN32/)
        target_link_libraries(multi_agent_systems PRIVATE ${OPENGL_LIBRARIES} glfw3.a)
    endif ()
    message("This system is Windows")
endif()

if (APPLE)
    message("This system is APPLE")
    set(CMAKE_CXX_FLAGS "-framework GLUT -framework OpenGL -Wno-deprecated -DGL_SILENCE_DEPRECATION ")
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/APPLE/)
    target_link_libraries(multi_agent_systems ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
elseif (UNIX)
    message("This system is UNIX")
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/UNIX/)
    target_link_libraries(multi_agent_systems ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif()
