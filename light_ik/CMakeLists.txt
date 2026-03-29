
set(HEADERS
    "headers/types.h"
    "headers/helpers.h"
    "headers/bone.h"
    "headers/skeleton.h"
    "headers/target.h"
    "headers/solver_base.h"
    "headers/solver.h"
    "headers/solver_passive.h"
)

set(SOURCES
    "src/light_ik.cpp"    
    "src/helpers.cpp"    
    "src/bone.cpp"
    "src/skeleton.cpp"
    "src/solver.cpp"
    "src/target.cpp"
)

find_package(glm REQUIRED)

add_library(light_ik STATIC ${HEADERS} ${SOURCES})

target_link_libraries(light_ik PUBLIC glm::glm)

target_include_directories(light_ik 
    PUBLIC ./include
    PRIVATE ./headers)
