#pragma once
#include "modelio/physical_scope/PhysicalScope.h"
#include "lib_src/assembly/NodalNodeDomain.h"

namespace crash::modelio::physical_domain {
enum class Policy { RetainedShellAssembliesV1, RetainedShellAssembliesExtendedSolidsV4, RetainedShellAssembliesVehicleSupportsV5,
    RetainedShellAssembliesNativeSupportsV6 };
enum class GroupDisposition { Complete, Restricted, Omitted };
struct GroupSelection {
    std::size_t source_group = SIZE_MAX;
    GroupDisposition disposition = GroupDisposition::Omitted;
    std::uint64_t case_node_set_id = 0;
    std::vector<std::uint64_t> members, excluded_members;
};
struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t domain_bytes = 32 * 1024 * 1024, topology_bytes = 8 * 1024 * 1024;
};
struct Forecast {
    std::size_t previous_phase = 0, retained_source = 0, decode_bytes = 0;
    std::size_t selection_bytes = 0, domain_reservation = 0, topology_reservation = 0;
    std::size_t current_phase = 0, total_bytes = 0;
};
struct Counts {
    std::size_t complete_groups = 0, restricted_groups = 0, omitted_groups = 0;
    std::size_t plain_members = 0, retained_point_masses = 0;
};
// Explicit original source reduction for the complete retained shell demo.
// Keeps every baseline/TYPE25 node and original PART member. Plain groups that
// intersect that geometry also retain their real original point-mass members.
// All other omitted members/cards remain source evidence. This creates source
// topology/domain only: no coefficients, constraints or coupled-case admission.
class VehiclePhysicalDomain {
  public:
    static Forecast Preflight(const physical_scope::PhysicalScope&, Policy, Limits = {});
    static VehiclePhysicalDomain Prepare(const physical_scope::PhysicalScope&, Policy, Limits = {});
    const physical_scope::PhysicalScope& source() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const tl::fea::rigid::NodalRigidPartTopology& topology() const noexcept;
    const std::vector<GroupSelection>& plain_groups() const noexcept;
    Counts counts() const noexcept;
    const Forecast& forecast() const noexcept;
    Policy policy() const noexcept;
  private:
    struct Storage;
    explicit VehiclePhysicalDomain(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::physical_domain
