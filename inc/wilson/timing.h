#pragma once

// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <concepts>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"
#include "absl/functional/function_ref.h"
#include "absl/strings/str_join.h"
#include "third_party/xxh64.h"

// A header-only library for run-time performance timers.  Contains support
// for timing operations even in very hot inner loops.  To use, either call
// a timeit method with the name of the timer and an optional tooltip:
//
// timeit("filling", "Time to fill polygons", [&]() {
//    <code>
// });
//
// Or, alternatively, create a Timer instance and call its timeit member.  This
// is useful if you're going to be running some operation in a loop repeatedly.
// Instantiating the timer does any runtime lookups once, and actually
// activating and deactivating the timer is very efficient:
//
// Timer fill_timer("filling, "Time to fill polygons");
// for (auto polygon : polygons) {
//   ... other stuff
//   fill_timer.timeit([&]() {
//     <fill polygon>
//   });
//   ... other stuff
// }
//
// Execution time is recorded and exponentially averaged.  The current value
// of the timers can be obtained through the VisitTimings function:
//
// VisitTimings([](absl::string_view name, absl::string_view tip, double time) {
//   ...
// });
//
// The timer name is built hierarchically from the active timers when the
// timer is first created.  Thus nested timers:
//
// timeit("foo", []() {
//   timeit("bar", []() {
//     timeit("baz", []() {
//     });
//   });
// });
//
// Will create three timers, foo, foo.bar and foo.bar.baz, each of which will
// record a separate time for its enclosing scope.
//
// To avoid any issues with multi-threading, each thread has a separate store
// for timers and timing data, proceed accordingly.
//
// The library works by computing a compile-time hash from the timer name.  Thus
// only literal timer names are supported.  This hash is a 64-bit integer which
// is used to represent the timer in subsequent operations.
//
// The timer data is stored in a flat_hash_map associated with this hash value
// and it's thus possible (but vanishingly unlikely) that two timers could
// theoretically map to the same performance value, yielding bad timing data.
// If this every verifiably happens I will buy the afflicted individual a Coke.
//
// Done this way, activating a timer only requires pushing two integers onto a
// stack, and deactivating popping the same.  This is what makes it possible to
// put these timers liberally throughout a graphics loop without significant
// performance impact.
namespace w {

// Returns the current wall clock time in seconds.
static inline double stopwatch() {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec + (double)tv.tv_nsec*1e-9;
}

// Returns the wall clock time elapsed from a starting time, in seconds.
static inline double stopwatch(double start) {
  return stopwatch()-start;
}

namespace internal {

// A helper class that will only bind to a string literal and computes a 64 bit
// hash value for it at compile time.
struct hashed_string {
  // A random seed, no significance.
  static constexpr uint64_t seed = 0x078a236743da49cb;

  template <typename T, size_t N> requires (std::same_as<T, const char>)
  consteval hashed_string(T (&str)[N])
    : data(str), hash(xxh64::hash(str, N, seed)) {}

  char const* data;
  uint64_t hash;
};

// Stores timing information
class TimingState {
public:
  static constexpr double kSmoothing = 0.10;

  using TimerId = uint64_t;

  // Registers a new timer with optional tooltip and returns a TimerId for it.
  //
  // Timers are only registered once.  Subsequent calls for the same timer name
  // will just return its TimerId.
  TimerId RegisterName(internal::hashed_string name, absl::string_view tip) {
    uint64_t hash = active_hash_ ^ name.hash;
    if (!timer_registry_.contains(hash)) {
      timer_registry_[hash] = TimerInfo{FullName(name.data), std::string(tip)};
      timers_.emplace_back(hash);
    }
    return hash;
  }

  // Activates a timer.  Further Timers added while active become descendants.
  void Activate(TimerId id) {
    timer_stack_.emplace_back(std::make_pair(active_hash_, id));
    active_hash_ = id;
  }

  // Deactivate the last activated timer (if any).
  void Deactivate() {
    if (!timer_stack_.empty()) {
      active_hash_ = timer_stack_.back().first;
      timer_stack_.pop_back();
    }
  }

  // Update timing information for the given timer.
  void UpdateTime(TimerId id, double newval) {
    double& currval = timings_[id];
    currval = kSmoothing*newval + (1-kSmoothing)*currval;
  }

  // Visit all the timer values and pass them to a callback.
  void VisitTimings(
    absl::FunctionRef<void(absl::string_view, absl::string_view, double)> visitor) {
    for (TimerId id : timers_) {
      const TimerInfo& info = timer_registry_[id];
      visitor(info.name, info.tip, timings_[id]);
    }
  }

private:
  // A hash representing the current active timer.
  uint64_t active_hash_ = 0;

  struct TimerInfo {
    std::string name;
    std::string tip;
  };

  // A mapping from hash -> real information about a timer.
  absl::flat_hash_map<uint64_t, TimerInfo> timer_registry_;

  // A deduplicated list of timers maintained in insertion-order.
  std::vector<uint64_t> timers_;

  // A stack of currently active timer ids.
  std::vector<std::pair<uint64_t, uint64_t>> timer_stack_;

  // The store of timing values for each timer id.
  absl::flat_hash_map<uint64_t, double> timings_;

  // Builds a full name for a timer based on the current stack of names.
  std::string FullName(absl::string_view name) {
    if (timer_stack_.empty()) {
      return std::string(name);
    }

    uint64_t ancestor = timer_stack_.back().second;
    return absl::StrCat(timer_registry_[ancestor].name, ".", name);
  }
};

// Thread local timing state.  Wrap it in NoDestructor to avoid issues with
// static destruction order being undefined.
thread_local absl::NoDestructor<TimingState> timing_results;
} // namespace internal

// A timer class that can be called multiple times.  Successive invocations are
// summed and the final result stored in the timer.  This can be a bit more
// efficient when we need to accumulate many results under one timer name in
// rapid succession, as it's able to lookup the timer's hash once instead of
// per-call.
class Timer {
public:
  Timer(const internal::hashed_string& name, absl::string_view tip="")
    : timing_state_(&*internal::timing_results) {
    timer_ = timing_state_->RegisterName(name, tip);
  }

  ~Timer() {
    timing_state_->UpdateTime(timer_, time_);
  }

  // Time one execution of yield.  Temporarily pushes the timer name onto the
  // timer stack while it's executing.
  void timeit(absl::FunctionRef<void(void)> yield) {
    timing_state_->Activate(timer_);
    double start = stopwatch();
    yield();
    time_ += stopwatch(start);
    timing_state_->Deactivate();
  }

private:
  internal::TimingState* timing_state_;
  internal::TimingState::TimerId timer_;
  double time_ = 0;
};

// Starts a new timing section.  Calls the given callback to yield control and
// handles automatically timing and registering the section.
inline void timeit(const internal::hashed_string& name,
  absl::FunctionRef<void(void)> yield) {
  Timer(name).timeit(yield);
}

// An overload of the above that takes a tooltip to display as well.
inline void timeit(const internal::hashed_string& name, absl::string_view tip,
  absl::FunctionRef<void(void)> yield) {
  Timer(name, tip).timeit(yield);
}

// Takes a callback and yields current timing information, passing the key, tip
// (if any), and currently timing value.
inline void VisitTimings(
  absl::FunctionRef<void(absl::string_view, absl::string_view, double)> cb) {
  internal::timing_results->VisitTimings(cb);
}

} // namespace w
