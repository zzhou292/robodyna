#pragma once

#include "VehicleSelfContactSetup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "lib_src/collision/SelfContactTransaction.h"

namespace crash::cases::vehicle_self_contact {

class CandidateRigidCouponAccess;

inline constexpr double FirstProfileStiffnessPerAreaNPerM3 = 2e9;

struct RuntimeConfig {
    // Both are mandatory app identities/capacities. Stiffness is deliberately
    // not caller-selectable in the first frictionless level-0 profile.
    std::uint64_t source_id = 0;
    std::size_t event_capacity = 0;
    unsigned broadphase_axis = 0;
    bool enable_diagnostics = false;
};

struct RuntimeLimits {
    std::size_t host_bytes = std::size_t{20} * 1000 * 1000 * 1000;
    std::size_t device_bytes = std::size_t{8} << 30;
    tlfea::contact::SelfContactTransactionLimits transaction;
};

struct RuntimeIdentity {
    std::uint64_t source_id = 0;
    std::uint64_t owner_id = 0;
    std::uint64_t configuration_id = 0;
    std::uint64_t qualification_id = 0;
    std::uint64_t physical_source_instance_id = 0;
    const void* active_use_identity = nullptr;
    std::size_t roster_entries = 0;
};

struct RuntimeForecast {
    tlfea::contact::SelfContactTransactionForecast transaction;
    RuntimeIdentity identity;
    std::size_t retained_host_upper_bound = 0;
    std::size_t peak_host_upper_bound = 0;
    std::size_t device_bytes = 0;
};

struct RuntimeSourceShape {
    unsigned facet_level = 0;
    std::size_t physical_nodes = 0;
    std::size_t selected_parents = 0;
    std::size_t fixed_facets = 0;
    std::size_t applied_original_friction_fields = 0;
    std::size_t applied_original_damping_fields = 0;
    std::size_t applied_original_soft_fields = 0;
};

namespace detail {
class SelfContactStages;
}
class SelfContactOnly;
class LoadedWallSelfContact;

// Focused runtime owner for the immutable vehicle selection and one private TL
// transaction. It always binds the exact existing physical owner/publication;
// it creates no second owner or clock.
class VehicleSelfContactStartup {
  public:
    static RuntimeSourceShape SourceShape(
        const VehicleSelfContactSetup& setup) noexcept {
        RuntimeSourceShape result;
        result.facet_level = setup.config().facet_level;
        result.physical_nodes =
            setup.physical().domain()->node_count();
        result.selected_parents =
            setup.active_uses().parents().size();
        result.fixed_facets =
            setup.active_uses().facet_uses().size();
        const auto coefficients =
            setup.census().runtime_coefficients;
        result.applied_original_friction_fields =
            coefficients.applied_source_friction_fields;
        result.applied_original_damping_fields =
            coefficients.applied_source_damping_fields;
        result.applied_original_soft_fields =
            coefficients.applied_source_soft_fields;
        return result;
    }
    static RuntimeForecast Preview(
        const VehicleSelfContactSetup&, vehicle_dynamics::Config,
        RuntimeConfig, RuntimeLimits,
        const vehicle_runtime::JointModel* = nullptr);
    static RuntimeForecast Preflight(
        const VehicleSelfContactSetup&,
        vehicle_dynamics::VehiclePhysicalDynamics&,
        RuntimeConfig, RuntimeLimits);
    // Focused self-only startup path. Runtime stepping uses SelfContactOnly,
    // which installs the same concrete object into VehiclePhysicalDynamics.
    static VehicleSelfContactStartup Prepare(
        const VehicleSelfContactSetup&,
        vehicle_dynamics::VehiclePhysicalDynamics&,
        RuntimeConfig, RuntimeLimits);
    ~VehicleSelfContactStartup();
    VehicleSelfContactStartup(VehicleSelfContactStartup&&) noexcept;
    VehicleSelfContactStartup& operator=(
        VehicleSelfContactStartup&&) noexcept;
    VehicleSelfContactStartup(const VehicleSelfContactStartup&) = delete;
    VehicleSelfContactStartup& operator=(
        const VehicleSelfContactStartup&) = delete;

    const RuntimeForecast& forecast() const noexcept;
    const VehicleSelfContactSetup& setup() const noexcept;
    const tl::fea::NodalStamp& initial_stamp() const noexcept;
    tlfea::contact::SelfContactTransactionAllocationInfo
        allocations() const noexcept;

  private:
    friend class SelfContactOnly;
    friend class LoadedWallSelfContact;
    friend class detail::SelfContactStages;
    friend class CandidateRigidCouponAccess;
    struct Data;
    static VehicleSelfContactStartup PrepareUnconfigured(
        const VehicleSelfContactSetup&,
        vehicle_dynamics::VehiclePhysicalDynamics&,
        RuntimeConfig, RuntimeLimits);
    tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
    explicit VehicleSelfContactStartup(std::unique_ptr<Data>);
    std::unique_ptr<Data> data_;
};

}  // namespace crash::cases::vehicle_self_contact
