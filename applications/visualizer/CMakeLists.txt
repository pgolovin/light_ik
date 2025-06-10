
set(HEADERS
    "plot_area.h"
    "window.h"
)

set(SOURCES
    "main.cpp"
    "plot_area.cpp"
    "window.cpp")

    

add_executable(visualizer ${HEADERS} ${SOURCES})
target_link_libraries(visualizer PUBLIC light_ik)