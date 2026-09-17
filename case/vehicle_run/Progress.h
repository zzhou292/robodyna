#pragma once
#include "case/vehicle_dynamics/StepTiming.h"
#include "MechanicsTotals.h"
#include "SampledShellPlasticityTotals.h"
#include "SelfContactTotals.h"
#include <cstdint>
#include <functional>
#include <string>
namespace crash::cases::vehicle_run {
struct Endpoint {std::uint64_t epoch=0;double time_s=0;};
struct Timing {double step_s=0,commit_s=0,archive_s=0,capture_s=0;};
struct ContactTotals {
    bool available=false;
    std::uint64_t intervals=0,accepted_active_parents=0,proposed_active_parents=0;
    double peak_observed_force_n=0,peak_observed_penetration_m=0,peak_observed_potential_j=0;
    double reported_drift_work_sum_j=0,last_same_mask_potential_j=0,last_removed_potential_j=0;
};
struct Progress {
    Endpoint accepted;
    std::uint64_t planned_intervals=0;
    double elapsed_s=0,accepted_intervals_per_second=0;
    Timing timing;
    ContactTotals contact;
    SelfContactTotals self_contact;
    MechanicsTotals mechanics;
    SampledShellPlasticityTotals sampled_shell_plasticity;
    vehicle_dynamics::StepTimingSnapshot mechanics_timing;
};
struct Control {
    std::uint64_t maximum_accepted_intervals=0; // Explicit short diagnostic prefix; zero disables.
    double maximum_elapsed_s=0; // Zero disables this cooperative boundary limit.
    double progress_period_s=5;
    std::function<bool()> stop_requested;
    std::function<void(const Progress&)> progress;
};
enum class StopKind { Completed, Requested, IntervalLimit, TimeLimit, StartupFailure, PhysicsRejected, ArchiveFailure, CaptureFailure, ObserverFailure };
struct LoopResult {
    StopKind kind=StopKind::PhysicsRejected;
    Progress progress;
    std::string reason;
    bool valid_manifest=false;
};
} // namespace crash::cases::vehicle_run
