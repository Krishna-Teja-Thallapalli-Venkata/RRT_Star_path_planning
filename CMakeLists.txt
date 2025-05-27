cmake_minimum_required(VERSION 3.10)
project(RRT_Star_path_planning)


set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 

include(CTest)
enable_testing()

#set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include directories
include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/rrt_star.cpp
    src/visualizer.cpp
    # src/point.cpp and src/node.cpp are not needed as they are header-only
)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# Set output directory for the executable
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Add compiler flags for debugging and optimization
if(MSVC)
    # MSVC-compatible flags
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
    # GCC/Clang-compatible flags
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${PROJECT_NAME} PRIVATE -g -O0 -Wall -Wextra -pedantic)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -O3 -DNDEBUG -Wall -Wextra -pedantic)
    endif()
endif()

# Doxygen creation can be done manually also
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/doxygen_docs)
    set(DOXYGEN_INPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
    set(DOXYGEN_CONFIG_FILE ${CMAKE_SOURCE_DIR}/Doxyfile)

    add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIG_FILE}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
    message(STATUS "Doxygen found. Documentation target 'docs' added.")
    message(STATUS "Run 'make docs' or 'ninja docs' to generate documentation in ${DOXYGEN_OUTPUT_DIRECTORY}")
else()
    message(WARNING "Doxygen not found. Documentation target 'docs' will not be available.")
endif()