
set(LIGHT_IK_TEST_HEADERS
    "test_helpers.h"
)

set(LIGHT_IK_TEST_SRC
    "main.cpp"
    "bone_test.cpp"
    "light_ik_test.cpp"
)

find_package(GTest REQUIRED)

add_executable(tests ${LIGHT_IK_TEST_SRC} ${LIGHT_IK_TEST_HEADERS})
target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests  
                        PUBLIC light_ik 
                        PUBLIC GTest::gtest
                        PUBLIC GTest::gtest_main)