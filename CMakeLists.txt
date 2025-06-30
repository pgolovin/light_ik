cmake_minimum_required(VERSION 3.27)
project(light_ik)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}_${ARCHITECTURE}")
string(REGEX REPLACE "/W[3|4]" "/w" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")

set(CMAKE_MSVC_RUNTIME_LIBRARY "$<$<CONFIG:Debug>:MultiThreadedDebugDLL>")

execute_process(COMMAND conan profile detect --force)
execute_process(COMMAND conan install ${PROJECT_SOURCE_DIR}\\3rd_party\\conan --output-folder=${CMAKE_BINARY_DIR} --build=missing -s build_type=${CMAKE_BUILD_TYPE})
add_compile_definitions(CMAKE_TOOLCHAIN_FILE="${CMAKE_BINARY_DIR}/conan_toolchain.cmake")

# add sub-project
add_subdirectory(${PROJECT_SOURCE_DIR}/applications/tests)
add_subdirectory(${PROJECT_SOURCE_DIR}/applications/visualizer)
add_subdirectory(${PROJECT_SOURCE_DIR}/light_ik)

