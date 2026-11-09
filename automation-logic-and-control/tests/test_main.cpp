// test_main.cpp
// Single entry point for the unit-test executable. Includes the two
// translation units' test runners and prints a final summary.

#include "test_framework.hpp"

#include <cstdio>

// Forward-declared test runners defined in their own .cpp files.
void runPidTests();
void runFsmTests();

int main() {
    std::printf("Starting automation-logic-and-control unit tests...\n\n");

    runPidTests();
    std::printf("\n");
    runFsmTests();

    TEST_SUMMARY();
    return TEST_FAILED() ? 1 : 0;
}
