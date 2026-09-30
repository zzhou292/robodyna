// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <array>
#include <cstdint>
#include <limits>
#include <ostream>

namespace filter_batch_benchmark {
struct Timing {
  std::size_t calls = 0;
  double total_s = 0, minimum_s = std::numeric_limits<double>::infinity(), maximum_s = 0;
  void Add(double seconds);
};
struct Counts {
  std::size_t valid = 0, invalid = 0, separated = 0;
  std::array<std::size_t, 7> categories{};
  std::array<std::size_t, 5> axes{};
};
struct Result {
  const char* name = "";
  bool accepted = false;
  c::SelfContactFacetPrismAxisLimit limit = c::SelfContactFacetPrismAxisLimit::FaceNormal;
  Timing cpu, gpu, warmup_cpu, warmup_gpu, verification;
  double reference_setup_s = 0;
  std::uint64_t digest = 0;
  std::array<Counts, ClassCount> classes;
};
std::uint64_t InputDigest(const Cases&);
void CheckReport(f::Report);
std::uint64_t Verify(const Cases&, bool accepted, c::SelfContactFacetPrismAxisLimit,
    const std::vector<f::PairResult>& cpu, f::ResultView gpu,
    const std::vector<f::PairResult>& reference, std::uint64_t scene_generation,
    std::array<Counts, ClassCount>* counts = nullptr);
void WriteTiming(std::ostream&, const Timing&);
void WriteResult(std::ostream&, const Result&, const Cases&);
}  // namespace filter_batch_benchmark
