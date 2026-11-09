// test_framework.hpp
//
// A zero-dependency, single-header test harness. The goal is "Data
// Verification" without pulling in a heavy external framework: a handful
// of CHECK / REQUIRE macros that record pass/fail counts and print
// readable diagnostics. This compiles on any Linux box with g++ -std=c++17,
// which keeps the repo self-contained for offline industrial environments.
//
// If you later want Catch2/Googletest, these assertions are the only
// call-sites you'd touch.

#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

namespace test_framework {

// Shared counters (one definition, inline, so one per T.U. is fine because
// they are intentionally not shared across translation units; the summary
// is printed from a single main()).
struct Counters {
    static int& total()   { static int n = 0; return n; }
    static int& failed()  { static int n = 0; return n; }
};

inline void report(bool ok, const char* expr, const char* file, int line) {
    ++Counters::total();
    if (ok) {
        std::printf("  [PASS] %s\n", expr);
    } else {
        ++Counters::failed();
        std::printf("  [FAIL] %s (%s:%d)\n", expr, file, line);
    }
}

} // namespace test_framework

#define CHECK(cond) \
    ::test_framework::report(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

#define CHECK_EQ(actual, expected) \
    ::test_framework::report(static_cast<bool>((actual) == (expected)), \
                             #actual " == " #expected, __FILE__, __LINE__)

// REQUIRE behaves like CHECK but aborts the test run on failure (useful for
// "this must be true or nothing else is meaningful" assertions).
#define REQUIRE(cond) \
    do { \
        ::test_framework::report(static_cast<bool>(cond), #cond, __FILE__, __LINE__); \
        if (!(cond)) std::abort(); \
    } while (0)

#define TEST_SUMMARY() \
    do { \
        std::printf("\n----------------------------------------\n"); \
        std::printf("Tests run: %d, Failures: %d\n", \
                    ::test_framework::Counters::total(), \
                    ::test_framework::Counters::failed()); \
        std::printf("----------------------------------------\n"); \
    } while (0)

#define TEST_FAILED() (::test_framework::Counters::failed() > 0)
