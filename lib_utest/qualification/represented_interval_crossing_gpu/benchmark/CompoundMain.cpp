// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CompoundResults.h"
#include "Selection.h"
#include <cuda_runtime_api.h>
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace b = native_batch_benchmark;
namespace q = native_gpu_benchmark;
namespace c = tlfea::contact;
namespace batch = c::represented_interval_crossing;
using Clock = std::chrono::steady_clock;
double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
struct Stream {
  cudaStream_t value = nullptr;
  void Initialize() {
    q::Require(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking) == cudaSuccess,
               "Benchmark CUDA stream creation failed");
  }
  ~Stream() { if (value) { cudaStreamSynchronize(value); cudaStreamDestroy(value); } }
};
int main(int argc, char** argv) try {
  q::Require(argc >= 5 && argc % 2 == 1, "Use --backend cpu|gpu --repeats N [--pairs N] [--view mixed|eligible] [--numeric-cohort 0|4096] [--device-workers N]");
  std::string_view backend, view = "mixed";
  unsigned repeats = 0, pairs = b::PairCount, workers = 4096, cohort = 0;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view name(argv[i]), text(argv[i+1]);
    if (name == "--backend") { backend = text; continue; }
    if (name == "--view") { view = text; continue; }
    unsigned* target = name == "--repeats" ? &repeats : name == "--pairs" ? &pairs :
        name == "--device-workers" ? &workers : name == "--numeric-cohort" ? &cohort : nullptr;
    q::Require(target != nullptr, "Unknown benchmark option");
    const auto parsed = std::from_chars(text.data(), text.data()+text.size(), *target);
    q::Require(parsed.ec == std::errc{} && parsed.ptr == text.data()+text.size(), "Invalid numeric option");
  }
  q::Require((backend == "cpu" || backend == "gpu") && (view == "mixed" || view == "eligible") &&
      repeats && repeats <= 10000 && workers && workers <= 4096 && (cohort == 0 || cohort == 4096) &&
      (backend == "gpu" || cohort == 0), "Invalid benchmark configuration");
  const auto setup = Clock::now();
  const auto source = b::MakeCases(pairs);
  auto limits = b::Limits(pairs);
  b::VerifiedResults original, expected;
  q::Selection eligible;
  {
    c::RepresentedIntervalCrossing oracle;
    q::Require(oracle.Initialize(limits).status == c::RepresentedIntervalStatus::Ok, "Raw CPU oracle initialization failed");
    const auto full = oracle.Certify(source.paths.data(), source.paths.size(), source.pairs.data(), source.pairs.size());
    original = b::Verify(source, full, oracle.results(), nullptr);
    eligible = q::Select(source, original, limits.max_depth, true);
    expected = original;
    if (view == "eligible") {
      const auto& subset = eligible.cases;
      const auto report = oracle.Certify(subset.paths.data(), subset.paths.size(), subset.pairs.data(), subset.pairs.size());
      expected = b::Verify(subset, report, oracle.results(), nullptr);
      b::VerifySubset(original, expected);
    }
  }
  const auto& cases = view == "eligible" ? eligible.cases : source;
  limits.max_input_pairs = limits.max_results = q::PublicationSlice;
  c::RepresentedIntervalCrossing cpu;
  q::Require(cpu.Initialize(limits).status == c::RepresentedIntervalStatus::Ok, "Compound CPU initialization failed");
  std::vector<c::RepresentedIntervalResult> reference_scratch(cases.pairs.size()), scratch(cases.pairs.size());
  const auto cpu_call = [&](auto& output) {
    return batch::BatchAccess::Certify(cpu, cases.paths.data(), cases.paths.size(),
        cases.pairs.data(), cases.pairs.size(), q::PublicationSlice, output.data(), output.size());
  };
  const auto control = cpu_call(reference_scratch);
  q::VerifyBatch(cases, expected, control, cpu.results());
  Stream stream;
  c::RepresentedIntervalCrossingGpu gpu;
  if (backend == "gpu") {
    stream.Initialize();
    c::RepresentedIntervalGpuLimits gpu_limits; gpu_limits.native = limits;
    gpu_limits.device_workers = workers; gpu_limits.numeric_cohort_pairs = cohort;
    const auto report = gpu.Initialize(gpu_limits, stream.value);
    q::Require(report.native.status == c::RepresentedIntervalStatus::Ok, report.native.message);
  }
  const auto setup_seconds = Seconds(setup);
  c::RepresentedIntervalDeviceReport device_control;
  double total = 0;
  for (unsigned i = 0; i < repeats + 2; ++i) {
    const auto start = Clock::now();
    batch::BatchReport actual;
    c::RepresentedIntervalDeviceReport device;
    if (backend == "gpu") {
      const auto value = batch::DeviceBatchAccess::Certify(gpu, cases.paths.data(), cases.paths.size(),
          cases.pairs.data(), cases.pairs.size(), q::PublicationSlice, scratch.data(), scratch.size(), stream.value);
      actual = value.native; device = value.device;
    } else actual = cpu_call(scratch);
    const auto elapsed = Seconds(start);
    q::VerifyBatch(cases, expected, actual, backend == "gpu" ? gpu.results() : cpu.results(), &control);
    if (backend == "gpu") {
      q::VerifyDevice(device, cases.pairs.size(), eligible.cases.pairs.size(), cohort,
          i ? &device_control : nullptr);
      device_control = device;
    }
    if (i >= 2) total += elapsed;
  }
  const auto forecast = gpu.forecast();
  std::cout << std::setprecision(17)
      << "{\"schema\":\"robo_dyna.native_gpu_compound_benchmark.v1\",\"backend\":\"" << backend
      << "\",\"view\":\"" << view << "\",\"source_pairs\":" << source.pairs.size()
      << ",\"selected_pairs\":" << cases.pairs.size() << ",\"omitted_pairs\":" << source.pairs.size()-cases.pairs.size()
      << ",\"paths\":" << cases.paths.size() << ",\"publication_slice\":" << q::PublicationSlice
      << ",\"native_pair_capacity\":" << limits.max_input_pairs << ",\"native_result_capacity\":" << limits.max_results
      << ",\"max_work_per_pair\":" << limits.max_work_per_pair << ",\"max_total_work\":" << limits.max_total_work
      << ",\"cpu_workers\":" << limits.worker_count << ",\"device_workers\":" << workers
      << ",\"numeric_cohort_pairs\":" << cohort << ",\"warmups\":2,\"repeats\":" << repeats
      << ",\"source_input_digest\":" << b::InputDigest(source) << ",\"input_digest\":" << b::InputDigest(cases)
      << ",\"source_work\":" << original.work << ",\"selected_work\":" << expected.work
      << ",\"omitted_work\":" << original.work-expected.work << ",\"result_digest\":" << expected.digest
      << ",\"last_native_report_digest\":" << b::NativeReportDigest(control.native_report)
      << ",\"batch_offset\":" << control.batch_offset << ",\"prior_work\":" << control.prior_work
      << ",\"completed_pairs\":" << control.completed_pairs << ",\"completed_batches\":" << control.completed_batches
      << ",\"path_authentications\":" << control.path_roster_work.authentications
      << ",\"path_rows\":" << control.path_roster_work.path_rows << ",\"vertex_rows\":" << control.path_roster_work.vertex_rows
      << ",\"path_sorts\":" << control.path_roster_work.path_sorts << ",\"vertex_sorts\":" << control.path_roster_work.vertex_sorts
      << ",\"device_pairs\":" << device_control.device_pairs << ",\"host_pairs\":" << device_control.host_pairs
      << ",\"consumed_device_pairs\":" << device_control.consumed_device_pairs << ",\"device_batches\":" << device_control.batches
      << ",\"scene_uploads\":" << device_control.scene_uploads << ",\"numeric_cohorts\":" << device_control.numeric_cohorts
      << ",\"fault_cohort_begin\":" << device_control.fault_cohort_begin << ",\"fault_cohort_count\":" << device_control.fault_cohort_count
      << ",\"fault_pair_ordinal\":" << device_control.fault_pair_ordinal << ",\"device_bytes\":" << forecast.device_bytes
      << ",\"numeric_cache_host_bytes\":" << forecast.numeric_cache_host_bytes
      << ",\"setup_s\":" << setup_seconds << ",\"total_certify_s\":" << total << ",\"mean_certify_s\":" << total/repeats << "}\n";
  return std::cout ? 0 : 1;
} catch (const std::exception& error) {
  std::cerr << "Compound native benchmark failed: " << error.what() << '\n'; return 1;
}
