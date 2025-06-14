
#set(HEADERS
#)

set(TYPES_SRC
    "src/light_ik_plugin.cpp"
    "src/register_types.cpp")

add_library(light_ik_plugin SHARED ${TYPES_SRC})

target_include_directories(light_ik_plugin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} PRIVATE ./src)
target_link_libraries(light_ik_plugin 
                        PUBLIC light_ik 
                        PUBLIC godot-cpp)

add_custom_command(TARGET light_ik_plugin POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin.gdextension ${PROJECT_SOURCE_DIR}/test_project/bin/)
add_custom_command(TARGET light_ik_plugin POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:light_ik_plugin> ${PROJECT_SOURCE_DIR}/test_project/bin/)