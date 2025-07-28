// Copyright 2025 Google LLC
// Author: smcallis@google.com (Sean McAllister)
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
#include "benchmark/benchmark.h"
#include "fuzztest/init_fuzztest.h"
#include "gtest/gtest.h"

int main(int argc, char* argv[]) {
  benchmark::Initialize(&argc, argv);
  fuzztest::ParseAbslFlags(argc, argv);
  fuzztest::InitFuzzTest(&argc, &argv);
  testing::InitGoogleTest(&argc, (char**) argv);

  // Just run benchmark if explicitly requested.
  if (!benchmark::GetBenchmarkFilter().empty()) {
    benchmark::RunSpecifiedBenchmarks();
    exit(0);
  }

  return RUN_ALL_TESTS();
}
