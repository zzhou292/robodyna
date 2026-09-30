#include "CouponProfileData.h"

#include <cuda_runtime_api.h>
#include <chrono>
#include <memory>
#include <stdexcept>

namespace crash::benchmarks {
namespace {
using Clock = std::chrono::steady_clock;
using case_data::ElasticCouponCase;
using output::Require;
using profile_detail::Data;

double Seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
void CheckCuda(cudaError_t status, const char* operation) {
    if (status != cudaSuccess)
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
}
void CheckCase(const case_data::CouponReport& report, const char* operation) {
    if (report.status != case_data::CouponStatus::Ok)
        throw std::runtime_error(std::string(operation) + ": " + report.diagnostic);
}
void MemorySample(Data& data, const char* phase, const ElasticCouponCase* run) {
    profile_detail::Memory sample;
    sample.phase = phase;
    std::size_t free = 0, total = 0;
    const auto start = Clock::now();
    const auto status = cudaMemGetInfo(&free, &total);
    sample.query_seconds = Seconds(start);
    CheckCuda(status, "cudaMemGetInfo");
    Require(free <= total, "Invalid CUDA memory observation");
    sample.free_bytes = free;
    sample.total_bytes = total;
    if (run) {
        sample.owner_initialized = run->metrics() != nullptr;
        if (sample.owner_initialized) sample.epoch = run->metrics()->stamp.epoch;
        const auto state = run->state_allocations();
        const auto elements = run->element_allocations();
        sample.owned_state_bytes = state.device_bytes;
        sample.owned_element_bytes = elements.device_bytes;
        sample.state_allocations = state.device_allocations;
        sample.element_allocations = elements.device_allocations;
    }
    data.memory.push_back(sample);
}
void Capture(Data& data, ElasticCouponCase& run) {
    const auto before = run.metrics()->stamp;
    const auto audits = run.metrics()->full_state_audit_reads;
    case_data::ElasticCouponFrame frame;
    const auto start = Clock::now();
    const auto report = run.Capture(frame);
    const double seconds = Seconds(start);
    CheckCase(report, "Capture");
    const auto& after = run.metrics()->stamp;
    Require(after.owner_id == before.owner_id && after.epoch == before.epoch &&
                output::Bits(after.time) == output::Bits(before.time) &&
                frame.stamp.owner_id == before.owner_id && frame.stamp.epoch == before.epoch &&
                output::Bits(frame.stamp.time) == output::Bits(before.time) &&
                run.metrics()->full_state_audit_reads == audits,
            "Capture changed accepted identity or returned a mismatched frame");
    data.captures.push_back({before.epoch, before.time, seconds});
}
void Steps(Data& data, ElasticCouponCase& run, unsigned count, bool warmup, unsigned repetition) {
    for (unsigned i = 0; i < count; ++i) {
        const auto before = run.metrics()->stamp;
        const auto audits = run.metrics()->full_state_audit_reads;
        const auto start = Clock::now();
        const auto report = run.Step();
        const double seconds = Seconds(start);
        CheckCase(report, "Step");
        const auto& metrics = *run.metrics();
        const auto& after = metrics.stamp;
        Require(after.owner_id == before.owner_id && after.epoch == before.epoch + 1 &&
                    output::Bits(after.fixed_dt) == output::Bits(before.fixed_dt) &&
                    output::Bits(after.time) == output::Bits(before.time + before.fixed_dt) &&
                    metrics.full_state_audit_reads >= audits && metrics.full_state_audit_reads <= audits + 1,
                "Accepted step identity or audit counter changed unexpectedly");
        data.steps.push_back({warmup, metrics.full_state_audit_reads != audits,
                              repetition, after.epoch, after.time, seconds});
    }
}
}  // namespace

void ValidateCouponProfileConfig(const CouponProfileConfig& config) {
    Require(config.warmup_steps >= 1 && config.warmup_steps <= 100,
            "Warmup steps must be 1..100");
    Require(config.repetitions >= 1 && config.repetitions <= 5,
            "Measured repetitions must be 1..5");
    Require(config.steps_per_repeat >= 1 && config.steps_per_repeat <= 300,
            "Steps per repetition must be 1..300");
    const auto total = std::uint64_t(config.warmup_steps) +
                       std::uint64_t(config.repetitions) * config.steps_per_repeat;
    Require(total <= 1000, "Profile is limited to 1000 total accepted steps");
}

output::Document ProfileElasticCoupon(const CouponProfileConfig& config) {
    ValidateCouponProfileConfig(config);
    Data data;
    data.config = config;
    const auto total = std::uint64_t(config.warmup_steps) +
                       std::uint64_t(config.repetitions) * config.steps_per_repeat;
    data.steps.reserve(total);
    data.captures.reserve(config.repetitions + 1);
    data.memory.reserve(2 * config.repetitions + 6);
    data.phases.reserve(5);

    // First CUDA API in this function. Sampling memory before it would itself
    // initialize CUDA and contaminate this interval. The guard owns that baseline.
    auto start = Clock::now();
    const auto initialized = cudaFree(nullptr);
    data.phases.push_back({"cuda_runtime_first_call", Seconds(start)});
    CheckCuda(initialized, "cudaFree(nullptr) runtime initialization");
    CheckCuda(cudaGetDevice(&data.device), "cudaGetDevice");
    cudaDeviceProp properties{};
    CheckCuda(cudaGetDeviceProperties(&properties, data.device), "cudaGetDeviceProperties");
    data.device_name = properties.name;
    CheckCuda(cudaRuntimeGetVersion(&data.runtime_version), "cudaRuntimeGetVersion");
    CheckCuda(cudaDriverGetVersion(&data.driver_version), "cudaDriverGetVersion");
    MemorySample(data, "after_cuda_runtime_initialization", nullptr);

    start = Clock::now();
    auto run = std::make_unique<ElasticCouponCase>();
    data.phases.push_back({"case_construction", Seconds(start)});
    MemorySample(data, "after_case_construction", run.get());
    start = Clock::now();
    const auto ready = run->Initialize({1, data.audit_intervals});
    data.phases.push_back({"case_initialize", Seconds(start)});
    CheckCase(ready, "Initialize");
    Require(run->metrics() && run->modal(), "Initialized case has no public metrics");
    Require(total < run->metrics()->required_steps && run->metrics()->stamp.epoch == 0,
            "Requested profile must remain a prefix of the admitted coupon horizon");
    data.modal = *run->modal();
    MemorySample(data, "after_case_initialize", run.get());
    Capture(data, *run);
    MemorySample(data, "after_initial_capture", run.get());

    Steps(data, *run, config.warmup_steps, true, 0);
    MemorySample(data, "after_warmup", run.get());
    for (unsigned repetition = 1; repetition <= config.repetitions; ++repetition) {
        Steps(data, *run, config.steps_per_repeat, false, repetition);
        const auto phase = "after_measured_repetition_" + std::to_string(repetition);
        MemorySample(data, phase.c_str(), run.get());
        Capture(data, *run);
        const auto captured = "after_capture_" + std::to_string(repetition);
        MemorySample(data, captured.c_str(), run.get());
    }
    data.final = *run->metrics();
    const auto& final = data.final;
    Require(final.stamp.epoch == total && final.diagnostics.valid &&
                final.diagnostics.owner_id == final.stamp.owner_id &&
                final.diagnostics.base_epoch + 1 == final.stamp.epoch &&
                final.diagnostics.configuration_id != 0 &&
                final.diagnostics.phase == tl::fea::reissner::ShellBatchPhase::kPreparedCandidate,
            "Final diagnostics do not describe the accepted profile endpoint");
    start = Clock::now();
    run.reset();
    data.phases.push_back({"case_destruction", Seconds(start)});
    MemorySample(data, "after_case_destroy", nullptr);
    return profile_detail::Report(data);
}
}  // namespace crash::benchmarks
