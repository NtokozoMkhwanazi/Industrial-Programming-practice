# CMakeLists.txt
# Build for the automation-logic-and-control portfolio.
#
#   cmake -S . -B build            # configure
#   cmake --build build            # build libraries + examples + tests
#   ./build/run_tests              # execute unit tests (the "Data Verification" step)
#   ./build/furnace_simulation    # run Project Idea 1 demo (ASCII)
#   ./build/furnace_dashboard     # run Project Idea 1 demo (SFML interactive dashboard)
#   ./build/conveyor_demo         # run Project Idea 2 demo

cmake_minimum_required(VERSION 3.10)

project(AutomationLogicAndControl
        VERSION 1.0.0
        DESCRIPTION "PID process control + FSM conveyor logic simulators"
        LANGUAGES CXX)

# --- C++17, no compiler extensions ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- Core static library: the reusable, unit-testable logic ---
add_library(automation_core STATIC
    src/pid/PIDController.cpp
    src/fsm/ConveyorFSM.cpp
)
target_include_directories(automation_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --- Unit-test executable (zero external deps; see tests/test_framework.hpp) ---
# NOTE: this harness is self-contained for offline/industrial environments.
# To swap in Catch2 later, replace tests/test_main.cpp and link Catch2WithMain.
enable_testing()
add_executable(run_tests
    tests/test_main.cpp
    tests/pid_tests.cpp
    tests/fsm_tests.cpp
)
target_link_libraries(run_tests PRIVATE automation_core)
target_include_directories(run_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
add_test(NAME automation_unit_tests COMMAND run_tests)

# --- Examples / demos ---
add_executable(furnace_simulation examples/furnace_simulation.cpp)
target_link_libraries(furnace_simulation PRIVATE automation_core)

add_executable(conveyor_demo examples/conveyor_demo.cpp)
target_link_libraries(conveyor_demo PRIVATE automation_core)

# Optional: enable extra warnings for a professional "clean build".
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(automation_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(run_tests        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(furnace_simulation PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(conveyor_demo     PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --- Optional SFML visualization (Project Idea 1 visual front-end) ---
# Uses find_package(SFML) provided by libsfml-dev on Debian/Ubuntu. The demo
# needs a real display to open its window, so it is opt-in and quietly skipped
# if SFML is unavailable (keeps the core repo build portable).
option(ENABLE_SFML "Build the SFML visualization demo" ON)
if(ENABLE_SFML)
    find_package(SFML 2.6 COMPONENTS system window graphics QUIET)
    if(SFML_FOUND)
        message(STATUS "SFML ${SFML_VERSION_MAJOR}.${SFML_VERSION_MINOR} found: building furnace_dashboard demo")
        add_executable(furnace_dashboard examples/furnace_dashboard.cpp)
        # Link the imported targets provided by SFMLConfig.cmake. sfml-graphics
        # is enough on its own (it pulls sfml-window -> sfml-system transitively),
        # but we list all three explicitly for clarity/portability.
        if(TARGET sfml-graphics)
            target_link_libraries(furnace_dashboard PRIVATE automation_core sfml-graphics sfml-window sfml-system)
        else()
            # Fallback to the variable form (module-mode FindSFML.cmake).
            target_link_libraries(furnace_dashboard PRIVATE automation_core ${SFML_LIBRARIES})
            target_include_directories(furnace_dashboard PRIVATE ${SFML_INCLUDE_DIRS})
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(furnace_dashboard PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    else()
        message(STATUS "SFML not found; skipping furnace_dashboard demo. (apt install libsfml-dev)")
    endif()
endif()
