// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Results.h"
#include "Selection.h"
#include "lib_src/collision/RepresentedIntervalCrossingGpu.h"
#include <cuda_runtime_api.h>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace b = native_batch_benchmark;
using Clock = std::chrono::steady_clock;
double Seconds(Clock::time_point begin) {
  return std::chrono::duration<double>(Clock::now() - begin).count();
}
struct Stream {
  cudaStream_t value = nullptr;
  void Initialize() {
    if (cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking) != cudaSuccess)
      throw std::runtime_error("CUDA stream creation failed");
  }
  ~Stream() { if (value) { cudaStreamSynchronize(value); cudaStreamDestroy(value); } }
};

int main(int argc, char** argv) try {
  if (argc < 5 || argc % 2 == 0)
    throw std::invalid_argument("Use --backend cpu|gpu --repeats N [--pairs N] [--device-workers N] [--view mixed|eligible]");
  std::string_view backend, view_name = "mixed";
  unsigned repeats = 0, pair_count = b::PairCount, device_workers = 128;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view option(argv[i]), value(argv[i + 1]);
    if (option == "--backend") { backend = value; continue; }
    if (option == "--view") { view_name = value; continue; }
    unsigned* destination = option == "--repeats" ? &repeats :
        option == "--pairs" ? &pair_count : option == "--device-workers" ? &device_workers : nullptr;
    if (!destination) throw std::invalid_argument("Unknown benchmark option");
    const auto parsed = std::from_chars(value.data(), value.data()+value.size(), *destination);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data()+value.size())
      throw std::invalid_argument("Invalid numerical benchmark option");
  }
  if (backend != "cpu" && backend != "gpu") throw std::invalid_argument("Unknown backend");
  if (view_name != "mixed" && view_name != "eligible") throw std::invalid_argument("Unknown diagnostic view");
  if (repeats == 0 || repeats > 10000 || !device_workers || device_workers > 4096)
    throw std::invalid_argument("Invalid repeat count or device worker count");
  const auto setup_start = Clock::now();
  const auto source = b::MakeCases(pair_count); const auto limits = b::Limits(pair_count);
  b::c::RepresentedIntervalCrossing reference_owner;
  auto report = reference_owner.Initialize(limits);
  if (report.status != b::c::RepresentedIntervalStatus::Ok) throw std::runtime_error(report.message);
  report = reference_owner.Certify(source.paths.data(), source.paths.size(), source.pairs.data(), source.pairs.size());
  const auto original = b::Verify(source, report, reference_owner.results(), nullptr);
  const auto selection = native_gpu_benchmark::Select(source, original, limits.max_depth, view_name == "eligible");
  const auto& cases = selection.cases;
  auto reference = original;
  if (view_name == "eligible") {
    report = reference_owner.Certify(cases.paths.data(), cases.paths.size(), cases.pairs.data(), cases.pairs.size());
    reference = b::Verify(cases, report, reference_owner.results(), nullptr);
    b::VerifySubset(original, reference);
    if (reference.work != selection.selected_work)
      throw std::runtime_error("Pair selection changed native proof work");
  }
  const double reference_setup_s = Seconds(setup_start);
  Stream stream; b::c::RepresentedIntervalCrossingGpu gpu;
  const auto owner_start = Clock::now();
  if (backend == "gpu") {
    stream.Initialize(); b::c::RepresentedIntervalGpuLimits gpu_limits; gpu_limits.native = limits;
    gpu_limits.device_workers = device_workers;
    const auto initialized = gpu.Initialize(gpu_limits, stream.value);
    if (initialized.native.status != b::c::RepresentedIntervalStatus::Ok)
      throw std::runtime_error(initialized.native.message);
  }
  const double owner_setup_s = Seconds(owner_start);
  b::c::RepresentedIntervalDeviceReport device_reference{};
  double total = 0, minimum = std::numeric_limits<double>::infinity(), maximum = 0;
  for (unsigned i = 0; i < repeats + 2; ++i) {
    const auto start = Clock::now();
    b::c::RepresentedIntervalResultView view;
    b::c::RepresentedIntervalDeviceReport device;
    if (backend == "gpu") {
      const auto result = gpu.Certify(cases.paths.data(), cases.paths.size(),
          cases.pairs.data(), cases.pairs.size(), stream.value);
      report = result.native; device = result.device; view = gpu.results();
    } else {
      report = reference_owner.Certify(cases.paths.data(), cases.paths.size(),
          cases.pairs.data(), cases.pairs.size());
      view = reference_owner.results();
    }
    const double elapsed = Seconds(start);
    b::Verify(cases, report, view, &reference);
    if (backend == "gpu") {
      if (device.status != b::c::RepresentedIntervalDeviceStatus::Ok || !device.device_pairs ||
          device.device_pairs + device.host_pairs != cases.pairs.size() || device.batches != 1 ||
          device.scene_uploads != 1)
        throw std::runtime_error("Device route is incomplete or unexercised");
      if (view_name == "eligible" && device.host_pairs)
        throw std::runtime_error("Eligible diagnostic unexpectedly routed a pair to CPU");
      if (i && (device.device_pairs != device_reference.device_pairs ||
                device.host_pairs != device_reference.host_pairs ||
                device.batches != device_reference.batches ||
                device.scene_uploads != device_reference.scene_uploads))
        throw std::runtime_error("Device route changed across repetitions");
      device_reference = device;
    }
    if (i >= 2) { total += elapsed; minimum = std::min(minimum, elapsed); maximum = std::max(maximum, elapsed); }
  }
  std::cout << std::setprecision(17)
      << "{\"schema\":\"robo_dyna.native_gpu_batch_benchmark.v1\",\"backend\":\"" << backend
      << "\",\"view\":\"" << view_name << "\",\"source_pairs\":" << source.pairs.size()
      << ",\"selected_pairs\":" << cases.pairs.size() << ",\"omitted_pairs\":" << source.pairs.size()-cases.pairs.size()
      << ",\"source_input_digest\":" << b::InputDigest(source)
      << ",\"source_work\":" << original.work << ",\"selected_work\":" << selection.selected_work
      << ",\"omitted_work\":" << selection.omitted_work
      << ",\"pairs\":" << cases.pairs.size() << ",\"paths\":" << cases.paths.size()
      << ",\"cpu_workers\":" << b::WorkerCount << ",\"device_workers\":" << device_workers
      << ",\"warmups\":2,\"repeats\":" << repeats
      << ",\"input_digest\":" << b::InputDigest(cases) << ",\"result_digest\":" << reference.digest
      << ",\"report_digest\":" << reference.report_digest << ",\"proof_work_per_batch\":" << reference.work
      << ",\"device_pairs\":" << device_reference.device_pairs << ",\"host_pairs\":" << device_reference.host_pairs
      << ",\"device_batches\":" << device_reference.batches
      << ",\"scene_uploads\":" << device_reference.scene_uploads
      << ",\"reference_setup_s\":" << reference_setup_s << ",\"owner_setup_s\":" << owner_setup_s
      << ",\"mean_certify_s\":" << total/repeats << ",\"minimum_certify_s\":" << minimum
      << ",\"maximum_certify_s\":" << maximum << ",\"total_certify_s\":" << total
      << ",\"device_bytes\":" << (backend == "gpu" ? gpu.forecast().device_bytes : 0)
      << ",\"classes\":[";
  for (unsigned i = 0; i < b::ClassCount; ++i) {
    const auto& value = reference.classes[i];
    std::cout << (i ? "," : "") << "{\"name\":\"" << b::Classes[i].name
        << "\",\"provenance\":\"" << b::Classes[i].provenance
        << "\",\"pairs\":" << value.separated + value.crossing + value.unresolved
        << ",\"separated\":" << value.separated
        << ",\"crossing\":" << value.crossing << ",\"unresolved\":" << value.unresolved
        << ",\"work\":" << value.work << "}";
  }
  std::cout << "]}\n";
  return std::cout ? 0 : 1;
} catch (const std::exception& error) {
  std::cerr << "Native GPU batch benchmark failed: " << error.what() << '\n'; return 1;
}
