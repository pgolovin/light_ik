
#set(HEADERS
#)

set(TYPES_SRC
    "main.cpp"
    "bone_test.cpp")

find_package(GTest REQUIRED)

add_executable(tests ${TYPES_SRC})
target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests  
                        PUBLIC light_ik 
                        PUBLIC GTest::gtest
                        PUBLIC GTest::gtest_main)