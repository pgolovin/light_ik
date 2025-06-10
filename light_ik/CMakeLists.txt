
set(HEADERS
    "headers/types.h"
    "headers/helpers.h"
    "headers/solver.h"
    "headers/bone.h"
)

set(SOURCES
    "src/helpers.cpp"    
    "src/solver.cpp"
    "src/bone.cpp")

find_package(glm REQUIRED)

add_library(light_ik STATIC ${HEADERS} ${SOURCES})

target_link_libraries(light_ik PUBLIC glm::glm)

target_include_directories(light_ik 
    PUBLIC ./include
    PRIVATE ./headers)
