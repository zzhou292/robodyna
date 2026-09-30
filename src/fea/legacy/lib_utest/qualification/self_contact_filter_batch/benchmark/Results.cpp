// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Results.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace filter_batch_benchmark {
namespace {
void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void Hash(std::uint64_t value, std::uint64_t& hash) noexcept {
  for (unsigned byte = 0; byte < 8; ++byte) {
    hash ^= value & 255; hash *= 1099511628211ULL; value >>= 8;
  }
}
std::uint64_t Bits(double value) noexcept {
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); return bits;
}
void Point(c::Vec3 value, std::uint64_t& hash) noexcept {
  Hash(Bits(value.x), hash); Hash(Bits(value.y), hash); Hash(Bits(value.z), hash);
}
}
void Timing::Add(double seconds) {
  Require(std::isfinite(seconds) && seconds >= 0, "Invalid benchmark clock sample");
  ++calls; total_s += seconds;
  minimum_s = std::min(minimum_s, seconds); maximum_s = std::max(maximum_s, seconds);
}
std::uint64_t InputDigest(const Cases& cases) {
  std::uint64_t hash = 1469598103934665603ULL;
  Hash(cases.values.accepted.size(), hash); Hash(cases.values.pairs.size(), hash);
  for (std::size_t facet = 0; facet < cases.values.accepted.size(); ++facet) {
    for (const auto vertex : cases.values.accepted[facet].vertices) Point(vertex, hash);
    for (const auto vertex : cases.values.prepared[facet].vertices) Point(vertex, hash);
    Hash(Bits(cases.values.properties[facet].half_thickness), hash);
    Hash(cases.values.properties[facet].complete_rigid_group, hash);
  }
  for (std::size_t pair = 0; pair < cases.values.pairs.size(); ++pair) {
    Hash(cases.values.pairs[pair].first, hash); Hash(cases.values.pairs[pair].second, hash);
    Hash(cases.kind[pair], hash);
  }
  return hash;
}
void CheckReport(f::Report report) {
  if (report.status != f::Status::Ok)
    throw std::runtime_error(std::string(report.message ? report.message : "Filter batch failed") +
        " status=" + std::to_string(unsigned(report.status)) + " pair=" + std::to_string(report.pair));
  Require(report.pair == SIZE_MAX && report.message && std::strcmp(report.message, "OK") == 0,
          "Successful batch report changed canonical fields");
}
std::uint64_t Verify(const Cases& cases, bool accepted, c::SelfContactFacetPrismAxisLimit limit,
    const std::vector<f::PairResult>& cpu, f::ResultView gpu,
    const std::vector<f::PairResult>& reference, std::uint64_t scene_generation,
    std::array<Counts, ClassCount>* counts) {
  Require(gpu.complete && gpu.data && gpu.count == cases.values.pairs.size() &&
              gpu.count == cpu.size() && cpu.size() == reference.size() &&
              gpu.scene_generation == scene_generation,
          "Filter result view or scene generation differs");
  std::uint64_t hash = 1469598103934665603ULL;
  Hash(gpu.count, hash); Hash(gpu.complete, hash);
  Hash(accepted, hash); Hash(static_cast<unsigned>(limit), hash);
  if (counts) *counts = {};
  for (std::size_t row = 0; row < gpu.count; ++row) {
    Require(filter_batch_test::Same(cpu[row], reference[row]) &&
                filter_batch_test::Same(gpu.data[row], reference[row]),
            "Complete CPU/GPU filter result fields differ");
    const auto& value = gpu.data[row];
    const auto category = static_cast<unsigned>(value.category), axis = static_cast<unsigned>(value.axis);
    Require(category < 7 && axis < 5, "Invalid published filter enum");
    Require(accepted ? (!value.separated && axis == 0)
                     : value.category == c::SelfContactFacetFilterCategory::ExactRemaining,
            "Unused accepted/linear result fields changed");
    Hash(row, hash); Hash(static_cast<unsigned>(value.status), hash);
    Hash(category, hash); Hash(value.separated, hash); Hash(axis, hash);
    if (counts) {
      auto& out = counts->at(cases.kind[row]);
      value.status == c::SelfContactFacetFilterStatus::Ok ? ++out.valid : ++out.invalid;
      out.separated += value.separated;
      ++out.categories[category]; ++out.axes[axis];
    }
  }
  return hash;
}
void WriteTiming(std::ostream& out, const Timing& value) {
  out << "{\"calls\":" << value.calls << ",\"total_s\":" << value.total_s
      << ",\"mean_s\":" << (value.calls ? value.total_s / value.calls : 0)
      << ",\"minimum_s\":" << (value.calls ? value.minimum_s : 0)
      << ",\"maximum_s\":" << value.maximum_s << '}';
}
void WriteResult(std::ostream& out, const Result& result, const Cases& cases) {
  out << "{\"name\":\"" << result.name << "\",\"result_digest\":" << result.digest
      << ",\"cpu_complete_batch\":"; WriteTiming(out, result.cpu);
  out << ",\"gpu_pair_upload_kernel_readback\":"; WriteTiming(out, result.gpu);
  out << ",\"warmup_cpu\":"; WriteTiming(out, result.warmup_cpu);
  out << ",\"warmup_gpu\":"; WriteTiming(out, result.warmup_gpu);
  out << ",\"reference_setup_s\":" << result.reference_setup_s;
  out << ",\"verification\":"; WriteTiming(out, result.verification);
  out << ",\"cpu_over_gpu\":";
  if (result.gpu.total_s > 0) out << result.cpu.total_s / result.gpu.total_s;
  else out << "null";
  out << ",\"classes\":[";
  for (unsigned kind = 0; kind < ClassCount; ++kind) {
    const auto& count = result.classes[kind];
    if (kind) out << ',';
    out << "{\"name\":\"" << Classes[kind].name << "\",\"pairs\":" << cases.counts[kind]
        << ",\"valid\":" << count.valid << ",\"invalid\":" << count.invalid
        << ",\"separated\":" << count.separated << ",\"categories\":[";
    for (unsigned i = 0; i < count.categories.size(); ++i) { if (i) out << ','; out << count.categories[i]; }
    out << "],\"axes\":[";
    for (unsigned i = 0; i < count.axes.size(); ++i) { if (i) out << ','; out << count.axes[i]; }
    out << "]}";
  }
  out << "]}";
}
}  // namespace filter_batch_benchmark
