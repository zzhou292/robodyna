// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Results.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string_view>

namespace b = filter_batch_benchmark;
using Clock = std::chrono::steady_clock;
namespace {
double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
std::size_t Number(const char* input, std::size_t maximum) {
  const auto text = std::string_view(input);
  std::size_t result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      !result || result > maximum)
    throw std::invalid_argument("Benchmark argument is outside its documented bound");
  return result;
}
struct Stream {
  cudaStream_t value = nullptr;
  Stream() {
    const auto error = cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking);
    if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
  }
  ~Stream() { if (value) cudaStreamDestroy(value); }
};

b::Result Measure(const b::Cases& cases, b::f::Batch& owner, cudaStream_t stream,
    const char* name, bool accepted, b::c::SelfContactFacetPrismAxisLimit limit,
    std::size_t repeats, std::uint64_t generation) {
  b::Result result;
  result.name = name; result.accepted = accepted; result.limit = limit;
  const auto setup_start = Clock::now();
  std::vector<b::f::PairResult> cpu(cases.values.pairs.size()), reference(cpu.size());
  b::EvaluateCpu(cases, accepted, limit, reference.data());
  // Reuse the owning corpus oracle once outside every timed batch. This checks
  // that the pre-expansion adapter calls the same public CPU certificate path.
  for (std::size_t row = 0; row < reference.size(); ++row)
    Require(filter_batch_test::Same(reference[row], filter_batch_test::Reference(
        cases.values, cases.values.pairs[row], accepted, limit)),
        "Pre-expanded CPU adapter differs from owning corpus reference");
  result.reference_setup_s = Seconds(setup_start);
  constexpr std::size_t Warmups = 2;
  for (std::size_t pass = 0; pass < Warmups + repeats; ++pass) {
    b::f::Report report;
    double cpu_s = 0, gpu_s = 0;
    const auto cpu_call = [&] {
      const auto start = Clock::now();
      b::EvaluateCpu(cases, accepted, limit, cpu.data());
      cpu_s = Seconds(start);
    };
    const auto gpu_call = [&] {
      const auto start = Clock::now();
      report = accepted ? owner.Accepted(cases.values.input(), stream)
                        : owner.Linear(cases.values.input(), limit, stream);
      gpu_s = Seconds(start);
    };
    // Paired ABBA order: CPU/GPU, GPU/CPU, GPU/CPU, CPU/GPU. No timers
    // include the following full field comparison or result serialization.
    const auto order = pass < Warmups ? pass : pass - Warmups;
    if (order % 4 == 0 || order % 4 == 3) { cpu_call(); gpu_call(); }
    else { gpu_call(); cpu_call(); }
    const auto verify_start = Clock::now();
    b::CheckReport(report);
    const auto digest = b::Verify(cases, accepted, limit, cpu, owner.results(), reference,
                                 generation, pass == 0 ? &result.classes : nullptr);
    if (pass == 0) result.digest = digest;
    else Require(digest == result.digest, "Repeated complete result digest changed");
    result.verification.Add(Seconds(verify_start));
    if (pass < Warmups) {
      result.warmup_cpu.Add(cpu_s); result.warmup_gpu.Add(gpu_s);
    } else {
      result.cpu.Add(cpu_s); result.gpu.Add(gpu_s);
    }
  }
  return result;
}
}  // namespace

int main(int argc, char** argv) try {
  std::size_t pairs = b::DefaultPairs, repeats = 20;
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << "self_contact_filter_batch_benchmark [--pairs N] [--repeats N]\n"
                 "Defaults: 4096 pairs, 20 repeats, 2 warmups. Pairs 1..65536; repeats 1..1000.\n"
                 "Scalar CPU versus complete CUDA batch, fixed retained scene; no vehicle stepping.\n";
    return 0;
  }
  bool seen_pairs = false, seen_repeats = false;
  for (int argument = 1; argument < argc; argument += 2) {
    if (argument + 1 == argc) throw std::invalid_argument("Missing benchmark argument value");
    const auto option = std::string_view(argv[argument]);
    if (option == "--pairs" && !seen_pairs) {
      pairs = Number(argv[argument + 1], b::MaximumPairs); seen_pairs = true;
    } else if (option == "--repeats" && !seen_repeats) {
      repeats = Number(argv[argument + 1], 1000); seen_repeats = true;
    } else throw std::invalid_argument("Use each of --pairs and --repeats at most once");
  }
  Require(b::f::CompatibleHostArithmetic(),
          "Benchmark requires unchanged host RN/gradual-underflow/masked-trap arithmetic");
  const auto source_start = Clock::now();
  const auto cases = b::MakeCases(pairs);
  const auto input_digest = b::InputDigest(cases);
  const double source_setup_s = Seconds(source_start);
  const auto limits = b::Limits(cases);
  const auto preflight = b::f::Batch::PreflightLimits(limits);
  b::CheckReport(preflight.report);
  const auto stream_start = Clock::now();
  Stream stream;
  const double stream_setup_s = Seconds(stream_start);
  b::f::Batch owner;
  const auto owner_start = Clock::now();
  b::CheckReport(owner.Initialize(limits, stream.value));
  const double owner_setup_s = Seconds(owner_start);
  const auto upload_start = Clock::now();
  b::CheckReport(owner.Upload(cases.values.scene(), stream.value));
  const double initial_scene_upload_s = Seconds(upload_start);
  std::uint64_t generation = 1;
  b::Timing scene_refresh;
  for (std::size_t pass = 0; pass < repeats; ++pass) {
    const auto start = Clock::now();
    const auto report = owner.Upload(cases.values.scene(), stream.value);
    const double elapsed = Seconds(start);
    b::CheckReport(report); ++generation;
    Require(!owner.results().complete, "Scene upload failed to revoke prior publication");
    scene_refresh.Add(elapsed);
  }
  const std::array<b::Result, 5> result{{
      Measure(cases, owner, stream.value, "accepted", true,
              b::c::SelfContactFacetPrismAxisLimit::VertexVertex, repeats, generation),
      Measure(cases, owner, stream.value, "linear_face_normal", false,
              b::c::SelfContactFacetPrismAxisLimit::FaceNormal, repeats, generation),
      Measure(cases, owner, stream.value, "linear_edge_cross", false,
              b::c::SelfContactFacetPrismAxisLimit::EdgeCross, repeats, generation),
      Measure(cases, owner, stream.value, "linear_vertex_edge", false,
              b::c::SelfContactFacetPrismAxisLimit::VertexEdge, repeats, generation),
      Measure(cases, owner, stream.value, "linear_vertex_vertex", false,
              b::c::SelfContactFacetPrismAxisLimit::VertexVertex, repeats, generation),
  }};
  const auto forecast = owner.forecast();
  Require(forecast.device_bytes == preflight.forecast.device_bytes &&
              forecast.owned_host_bytes == preflight.forecast.owned_host_bytes &&
              forecast.device_allocations == 1,
          "Persistent filter owner forecast changed during benchmark");
  std::cout.imbue(std::locale::classic());
  std::cout << std::setprecision(17)
      << "{\n\"schema\":\"robo_dyna.cuda_filter_batch_benchmark.v1\",\n"
      << "\"scope\":\"scalar public CPU certificates versus standalone CUDA numerical adapter; no vehicle stepping or contact authority\",\n"
      << "\"gpu_timing_scope\":\"pair admission/upload plus kernel and complete readback/sync; scene upload measured separately; not kernel-only time\",\n"
      << "\"corpus_scope\":\"fixed mixed scene with independent ordinal clones; zero source keys are not physical owner identities\",\n"
      << "\"all_results_equal\":true,\"pairs\":" << pairs << ",\"facets\":" << cases.values.accepted.size()
      << ",\"cpu_workers\":1,\"warmups\":2,\"repeats\":" << repeats
      << ",\"input_digest\":" << input_digest << ",\"scene_generation\":" << generation
      << ",\"source_setup_s\":" << source_setup_s << ",\"stream_setup_s\":" << stream_setup_s
      << ",\"owner_setup_s\":" << owner_setup_s << ",\"initial_scene_upload_s\":" << initial_scene_upload_s
      << ",\"scene_refresh\":"; b::WriteTiming(std::cout, scene_refresh);
  std::cout << ",\"case_host_payload_bytes\":" << cases.HostPayloadBytes()
      << ",\"owner_host_bytes\":" << forecast.owned_host_bytes
      << ",\"owner_startup_host_bytes\":" << forecast.startup_host_bytes
      << ",\"device_bytes\":" << forecast.device_bytes
      << ",\"device_allocations\":" << forecast.device_allocations
      << ",\"generic_seed_pairs\":" << cases.seed_pairs
      << ",\"yaris_geometry_included\":" << (cases.yaris_geometry ? "true" : "false");
#ifdef TL_FILTER_YARIS_FIXTURES
  std::cout << ",\"fixture_commit\":\"" << TL_FILTER_YARIS_COMMIT
      << "\",\"yaris_geometry_sha256\":\"" << TL_FILTER_YARIS_GEOMETRY_SHA256
      << "\",\"path_fixture_sha256\":\"" << TL_FILTER_YARIS_PATH_SHA256 << '"';
#endif
  std::cout << ",\"classes\":[";
  for (unsigned kind = 0; kind < b::ClassCount; ++kind) {
    if (kind) std::cout << ',';
    std::cout << "{\"name\":\"" << b::Classes[kind].name << "\",\"pairs\":" << cases.counts[kind]
        << ",\"provenance\":\"" << b::Classes[kind].provenance << "\"}";
  }
  std::cout << "],\n\"measurements\":[\n";
  for (unsigned phase = 0; phase < result.size(); ++phase) {
    if (phase) std::cout << ",\n";
    b::WriteResult(std::cout, result[phase], cases);
  }
  std::cout << "\n]}\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << "filter batch benchmark failed: " << error.what() << '\n';
  return 1;
}
