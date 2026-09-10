#pragma once
#include "SourceAssemblyObservation.h"
#include "lib_src/constraints/NodalRigidForceStageObservationTypes.h"
#include "lib_src/solvers/NodalForceStageSnapshot.h"

namespace crash::cases::source_assembly_observation {
struct ForceStageInput {
    const SourceAssemblyBindings* bindings=nullptr;
    // Actual nodal accepted readback stamp and separately returned group stamp.
    fe::NodalStamp base,before_group_stamp;
    fe::HostNodalKinematicsView before; // Only V/VR are read; x/q may be null and are ignored.
    const fe::NodalRigidGroupSnapshot *before_groups=nullptr,*force_groups=nullptr;
    std::size_t group_count=0;
    // Canonical owner candidate and the two complete readback associations.
    fe::NodalPreparedView prepared,frame_prepared,capture_prepared;
    const double *acceleration_xyz=nullptr,*angular_acceleration_xyz=nullptr;
    const fe::NodalRigidGroupAccelerationSnapshot* group_acceleration=nullptr;
    std::size_t acceleration_nodes=0,acceleration_groups=0;
};
struct ForceStageSummary {
    std::uint64_t owner_id=0,base_epoch=0,attempt=0,enclosing_epoch=0;
    fe::NodalRigidGroupInfo source;
    rigid::ForceStageObservationPhase phase;
    double enclosing_time=0;
    rigid::MemberKineticChannels ordinary,grouped_members;
    rigid::AggregateKineticChannels groups;
    ConnectorKineticChannels connector;
    double native_total=0,effective_total=0;
    // Sum group replacements directly; never subtract whole-assembly totals.
    double replacement=0;
};

// Pure supplied-value adapter. The caller authenticates live owner/token
// readbacks; this function checks complete readback/source/phase association.
// Pre-kick V/VR + actual A/AR at DT1/2, in the prepared group's updated frame.
// No force evaluation, state/clock ownership, allocation, midpoint-publication
// comparison or physical-energy acceptance tolerance. Output and all inputs
// remain unchanged on failure. Output must be disjoint from every inspected input.
Report ObserveForceStage(const ForceStageInput&,ForceStageSummary*) noexcept;
} // namespace crash::cases::source_assembly_observation
