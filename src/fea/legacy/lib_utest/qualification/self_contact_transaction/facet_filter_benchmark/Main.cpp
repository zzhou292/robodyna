// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Evaluate.h"
#include "Results.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>

namespace b = facet_filter_benchmark;
using Clock = std::chrono::steady_clock;
double Seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }
struct Stream {
  cudaStream_t value = nullptr;
  void Initialize() {
    b::Require(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking) == cudaSuccess, "CUDA stream creation failed");
  }
  ~Stream() { if (value) { cudaStreamSynchronize(value); cudaStreamDestroy(value); } }
};
int main(int argc, char** argv) try {
  b::Require(argc >= 5 && argc % 2 == 1,
      "Use --backend scalar|adapter --pattern all_linear|alternating|short_islands|no_linear [--pairs N] [--repeats N]");
  std::string_view backend, pattern;
  unsigned pairs = 4096, repeats = 100;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view option(argv[i]), value(argv[i + 1]);
    if (option == "--backend") { backend = value; continue; }
    if (option == "--pattern") { pattern = value; continue; }
    unsigned* target = option == "--pairs" ? &pairs : option == "--repeats" ? &repeats : nullptr;
    b::Require(target != nullptr, "Unknown benchmark option");
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), *target);
    b::Require(parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size(), "Invalid numeric option");
  }
  b::Require((backend == "scalar" || backend == "adapter") && repeats && repeats <= 10000,
      "Invalid backend or repeat count");
  b::Require(b::f::CompatibleHostArithmetic(), "Benchmark requires the qualified host arithmetic environment");
  const auto setup = Clock::now();
  b::Scene scene;
  scene.Initialize(pairs, pattern);
  std::vector<b::Row> reference(pairs), output(pairs);
  b::Evaluate(scene, nullptr, reference);
  for (const auto& row : reference)
    if (row.action == b::sct::PairMotionAction::LinearNodalV1)
      b::Require(row.numerical.status == b::c::SelfContactFacetFilterStatus::Ok, "Scalar fixture geometry is invalid");
  Stream stream;
  b::sct::FacetFilters adapter;
  const auto forecast = b::sct::FacetFilters::Preflight(scene.accepted.size(), pairs, 16u << 20, 16u << 20);
  b::Require(forecast.report.status == b::f::Status::Ok, forecast.report.message);
  if (backend == "adapter") {
    stream.Initialize();
    const auto initialized = adapter.Initialize(scene.source.uses, pairs, 16u << 20, 16u << 20, stream.value);
    b::Require(initialized.status == b::f::Status::Ok, initialized.message);
    b::Require(adapter.initialization_mode() == b::c::SelfContactFacetFilterInitialization::Cuda,
        "Adapter did not initialize its CUDA path");
    const auto ready = adapter.CandidateScene(scene.accepted.data(), scene.prepared.data(),
        scene.motion.data(), scene.bounds.data());
    b::Require(ready.status == b::f::Status::Ok, ready.message);
  }
  const double setup_s = Seconds(setup);
  b::Copies counts;
  double total = 0, minimum = std::numeric_limits<double>::infinity(), maximum = 0;
  for (unsigned i = 0; i < repeats + 2; ++i) {
    b::CopyScope observation;
    const auto start = Clock::now();
    b::Evaluate(scene, backend == "adapter" ? &adapter : nullptr, output);
    const double elapsed = Seconds(start);
    const auto actual = b::EndCopyObservation();
    b::Verify(scene, reference, output, actual, backend == "adapter");
    if (i) b::SameCopies(counts, actual);
    counts = actual;
    if (i >= 2) { total += elapsed; minimum = std::min(minimum, elapsed); maximum = std::max(maximum, elapsed); }
  }
  b::Require(!::testing::UnitTest::GetInstance()->ad_hoc_test_result().Failed(), "Qualification fixture assertion failed");
  std::cout << std::setprecision(17)
      << "{\"schema\":\"robo_dyna.facet_filter_adapter_benchmark.v1\",\"backend\":\"" << backend
      << "\",\"pattern\":\"" << pattern << "\",\"source_commit\":\"" << TL_FILTER_SOURCE_COMMIT
      << "\",\"begin_api\":\"" << b::BeginApi<b::sct::FacetFilters>()
      << "\",\"pairs\":" << pairs << ",\"facets\":" << scene.accepted.size()
      << ",\"linear_rows\":" << scene.linear_rows << ",\"nonlinear_rows\":" << scene.nonlinear_rows
      << ",\"excluded_rows\":" << scene.excluded_rows << ",\"input_digest\":" << b::InputDigest(scene)
      << ",\"result_digest\":" << b::ResultDigest(reference) << ",\"warmups\":2,\"repeats\":" << repeats
      << ",\"host_to_device_calls_per_iteration\":" << counts.host_to_device_calls
      << ",\"device_to_host_calls_per_iteration\":" << counts.device_to_host_calls
      << ",\"host_to_device_bytes_per_iteration\":" << counts.host_to_device_bytes
      << ",\"device_to_host_bytes_per_iteration\":" << counts.device_to_host_bytes
      << ",\"minimum_query_pairs\":" << counts.minimum_query_pairs
      << ",\"maximum_query_pairs\":" << counts.maximum_query_pairs
      << ",\"owned_host_bytes\":" << (backend == "adapter" ? forecast.owned_host_bytes : 0)
      << ",\"device_bytes\":" << (backend == "adapter" ? forecast.device_bytes : 0)
      << ",\"setup_s\":" << setup_s << ",\"total_query_s\":" << total
      << ",\"mean_query_s\":" << total / repeats << ",\"minimum_query_s\":" << minimum
      << ",\"maximum_query_s\":" << maximum << "}\n";
  return std::cout ? 0 : 1;
} catch (const std::exception& error) {
  std::cerr << "Facet-filter benchmark: " << error.what() << '\n'; return 1;
}
