#pragma once
// Minimal test harness (no dependencies).

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace gctest {

struct TestCase {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}
inline int& checks() {
  static int c = 0;
  return c;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline int runAll(const char* filter) {
  int failedTests = 0, ran = 0;
  for (const TestCase& t : registry()) {
    if (filter && std::string(t.name).find(filter) == std::string::npos) continue;
    const int before = failures();
    t.fn();
    ++ran;
    const bool ok = failures() == before;
    if (!ok) ++failedTests;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", t.name);
  }
  std::printf("\n%d tests, %d failed, %d checks, %d check failures\n", ran, failedTests, checks(), failures());
  return failedTests == 0 ? 0 : 1;
}

}  // namespace gctest

#define GC_CAT2(a, b) a##b
#define GC_CAT(a, b) GC_CAT2(a, b)
#define TEST_CASE(name)                                                          \
  static void GC_CAT(gc_test_fn_, __LINE__)();                                   \
  static gctest::Registrar GC_CAT(gc_test_reg_, __LINE__)(name, &GC_CAT(gc_test_fn_, __LINE__)); \
  static void GC_CAT(gc_test_fn_, __LINE__)()

#define CHECK(cond)                                                                  \
  do {                                                                               \
    ++gctest::checks();                                                              \
    if (!(cond)) {                                                                   \
      ++gctest::failures();                                                          \
      std::printf("  %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);         \
    }                                                                                \
  } while (0)

#define CHECK_EQ(a, b)                                                                              \
  do {                                                                                              \
    ++gctest::checks();                                                                             \
    const auto va_ = (a);                                                                           \
    const auto vb_ = (b);                                                                           \
    if (!(va_ == vb_)) {                                                                            \
      ++gctest::failures();                                                                         \
      std::printf("  %s:%d: CHECK_EQ(%s, %s) failed: %g vs %g\n", __FILE__, __LINE__, #a, #b,       \
                  static_cast<double>(va_), static_cast<double>(vb_));                              \
    }                                                                                               \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                       \
  do {                                                                                              \
    ++gctest::checks();                                                                             \
    const double va_ = static_cast<double>(a);                                                      \
    const double vb_ = static_cast<double>(b);                                                      \
    if (!(std::fabs(va_ - vb_) <= (tol))) {                                                         \
      ++gctest::failures();                                                                         \
      std::printf("  %s:%d: CHECK_NEAR(%s, %s) failed: %g vs %g (tol %g)\n", __FILE__, __LINE__, #a, \
                  #b, va_, vb_, static_cast<double>(tol));                                          \
    }                                                                                               \
  } while (0)
