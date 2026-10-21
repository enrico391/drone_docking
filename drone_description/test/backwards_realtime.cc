// Copyright 2026 Drone Docking contributors
// SPDX-License-Identifier: MIT
// Test-only clock fault injection. Never changes the kernel or host clock.

#include <cmath>
#include <ctime>
#include <dlfcn.h>

extern "C" int clock_gettime(clockid_t clock, struct timespec *value) noexcept
{
  using GetTime = int (*)(clockid_t, struct timespec *);
  static const auto getTime = reinterpret_cast<GetTime>(
    dlsym(RTLD_NEXT, "clock_gettime"));
  const int result = getTime(clock, value);
  if (result != 0 || clock != CLOCK_REALTIME) {
    return result;
  }

  struct timespec monotonic{};
  getTime(CLOCK_MONOTONIC, &monotonic);
  const double now = monotonic.tv_sec + monotonic.tv_nsec * 1e-9;
  static const double start = now;
  // Move UTC backwards faster than it advances, leaving monotonic time intact.
  value->tv_sec -= static_cast<time_t>(2 * std::floor((now - start) / 0.5));
  return result;
}
