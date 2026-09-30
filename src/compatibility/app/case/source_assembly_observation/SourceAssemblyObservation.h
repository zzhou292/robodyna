#pragma once

#include "case/source_assembly/SourceAssemblyBindings.h"
#include "lib_src/constraints/NodalRigidObservationTypes.h"
#include "lib_src/elements/ShellBatchPublication.h"

namespace crash::cases::source_assembly_observation {
namespace fe = tl::fea;
namespace rigid = fe::rigid;
using source_assembly::SourceAssemblyBindings;

enum class Status { Ok, InvalidInput, WrongIdentity, InvalidPhase, InvalidMetric,
                    NonfiniteResult, KineticMismatch, KickMismatch };
struct Report {
    Status status = Status::InvalidInput;
    const char* message = "Invalid assembly observation";
    std::size_t group = SIZE_MAX, node = SIZE_MAX;
    unsigned dof = 6;
    double residual = 0, roundoff_budget = 0;
    explicit operator bool() const noexcept { return status == Status::Ok; }
};
struct ConnectorKineticChannels {
    // These contributions are already included in ordinary translation/native
    // rotation. They are not shell physical J or artificial drilling J.
    double translation=0,rotation=0;
};
struct KineticSummary {
    rigid::ObservationPhase phase;
    rigid::MemberKineticChannels ordinary, grouped_members;
    rigid::AggregateKineticChannels groups;
    ConnectorKineticChannels connector;
    // Accumulate the disjoint ordinary-node and group partitions directly.
    // Native TOTAL J is authoritative; physical/added values are diagnostics.
    double native_total = 0, effective_total = 0;
    double publication_residual = 0, publication_roundoff_budget = 0;
};
struct Summary {
    KineticSummary before, after;
    rigid::KickWorkChannels applied, reaction;
    double native_delta = 0, effective_delta = 0, replacement_delta = 0;
    // Effective residual is native kick bookkeeping plus replacement/rounding,
    // not an independent aggregate-dynamics or full shell/contact energy proof.
    double native_residual = 0, effective_residual = 0, roundoff_budget = 0;
};
struct Input {
    const SourceAssemblyBindings* bindings = nullptr;
    fe::NodalStamp base;
    fe::NodalPreparedView prepared;
    fe::HostNodalKinematicsView before, after;
    const fe::NodalRigidGroupSnapshot* before_groups = nullptr;
    const fe::NodalRigidGroupSnapshot* after_groups = nullptr;
    std::size_t group_count = 0;
    // Exact global assembled load snapshot before SealAssembly, interleaved XYZ.
    const double* applied_force_xyz = nullptr;
    const double* applied_couple_xyz = nullptr;
    // Actual trial reactions from owner.CopyPrepared, never reconstructed loads.
    const double* reaction_force_xyz = nullptr;
    const double* reaction_couple_xyz = nullptr;
    fe::ShellBatchKinetic base_kinetic, kinetic;
};

// Pure bounded host observations. Caller authenticates live owner/token readbacks
// and the material/source binding before supplying these borrowed values.
// These functions check their complete source/phase association, allocate nothing,
// own no state/clock, and preserve output on failure. Shell/contact work and
// geometry admission belong to the case, not this kinetic observation adapter.
Report ObserveInitial(const SourceAssemblyBindings&, const fe::NodalStamp&,
                      fe::HostNodalKinematicsView,
                      const fe::NodalRigidGroupSnapshot*, std::size_t group_count,
                      const fe::ShellBatchKinetic&, KineticSummary*) noexcept;
Report ObserveInterval(const Input&, Summary*) noexcept;
} // namespace crash::cases::source_assembly_observation
