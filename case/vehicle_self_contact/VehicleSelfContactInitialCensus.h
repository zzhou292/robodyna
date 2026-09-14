#pragma once

#include "InitialCensusValues.h"
#include "VehicleSelfContactSetup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "lib_src/collision/SelfContactBroadphase.h"
#include <climits>
#include <stdexcept>
#include <string>

namespace crash::cases::vehicle_self_contact {

struct InitialCensusLimits {
    std::size_t max_pairs = INT_MAX;
    std::size_t max_host_bytes = std::size_t{2} << 30;
    std::size_t max_broadphase_device_bytes = std::size_t{2} << 30;
    unsigned broadphase_axis = 0;
};

struct InitialCensusSourceIdentity {
    std::uint64_t physical_domain_source_instance_id = 0;
    std::uint64_t owner_id = 0;
    std::uint64_t configuration_id = 0;
    std::uint64_t qualification_id = 0;
    std::uint64_t surface_active_source_hash = 0;
    std::size_t nodes = 0;
    std::size_t surface_parents = 0;
    std::size_t active_parents = 0;
    std::size_t physical_participants = 0;
    std::size_t rigid_groups = 0;
    std::size_t rigid_members = 0;
    std::size_t cin_rows = 0;
    std::size_t cin_witnesses = 0;
    bool exact_setup_physical_identity = false;
    bool exact_owner_stream_identity = false;
    bool exact_rigid_identity = false;
    bool exact_cin_identity = false;
    bool exact_participant_source_identity = false;
};

struct InitialCensusForecast {
    tlfea::contact::SelfContactBroadphaseForecast count_probe;
    tlfea::contact::SelfContactBroadphaseForecast cap_minus_one;
    tlfea::contact::SelfContactBroadphaseForecast exact;
    std::size_t surface_to_active_host_bytes = 0;
    std::size_t active_parent_host_bytes = 0;
    std::size_t pair_key_host_bytes = 0;
    std::size_t fixed_workspace_host_bytes = 0;
    std::size_t app_fixed_host_bytes = 0;
    // Exact sequential census reservation. Broadphase host fields retain the
    // source charge defined by that TL module; this is not a process RSS claim.
    std::size_t peak_census_host_reservation_bytes = 0;
    std::size_t retained_setup_host_reservation_bytes = 0;
    std::size_t physical_dynamics_peak_host_upper_bound = 0;
    // Explicit module-owned allocations only. CUDA runtime/driver storage is
    // outside the owning module APIs and is not inferred here.
    std::size_t physical_owner_device_bytes = 0;
    std::size_t peak_census_device_bytes = 0;
    std::size_t peak_total_explicit_device_bytes = 0;
};

struct InitialCensusResult {
    InitialCensusSourceIdentity source;
    InitialCensusForecast forecast;
    InitialFacetCapacityCensus capacity;
    std::uint64_t probe_required_pairs = 0;
    std::uint64_t cap_minus_one_required_pairs = 0;
    std::uint64_t exact_pair_count = 0;
    std::uint64_t rerun_pair_key_hash = 0;
    bool count_before_cap_observed = false;
    bool cap_minus_one_reproduced_required_count = false;
    bool exact_capacity_succeeded = false;
    bool complete_device_pair_keys = false;
    bool deterministic_rerun = false;
    bool accepted_owner_unchanged = false;
};

class InitialCensusCapacityError : public std::runtime_error {
  public:
    InitialCensusCapacityError(std::uint64_t required,
                               const std::string& message)
        : std::runtime_error(message), required_pairs_(required) {}
    std::uint64_t required_pairs() const noexcept {
        return required_pairs_;
    }

  private:
    std::uint64_t required_pairs_ = 0;
};

// One initial, current-motion count gate over the already-created dynamics
// owner. It creates no model, physical binding, participant, attachment or
// second state owner. The opened owner trial is always discarded; no force,
// feature, intersection or interval publication is attempted.
class VehicleSelfContactInitialCensus {
  public:
    static InitialCensusResult Measure(
        const VehicleSelfContactSetup&,
        vehicle_dynamics::VehiclePhysicalDynamics&,
        InitialCensusLimits = {});
};

}  // namespace crash::cases::vehicle_self_contact
