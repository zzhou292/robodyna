#pragma once
#include "SourcePartElasticInternal.h"
#include "case/source_part_wall/SourcePartWallContact.h"

namespace crash::cases::source_part_elastic {
namespace wall_contact=tlfea::contact;
std::array<double,3> CarriedAngularMomentum(const fe::ShellBatchBinding&,const Snapshot&) noexcept;
// Optional mechanics contributor and case-owned output caches. None owns a
// nodal state, material history, time integration or publication authority.
struct SourcePartWallState {
    source_part_wall::SourcePartWallContact contributor;
    wall_contact::NodalWallDiagnostics base;
    wall_contact::NodalWallDeviceResults trial,accepted;
    WallMetrics trial_metrics,accepted_metrics;
    std::array<double,NodeCount> endpoint_force_uncertainty{};
    bool has_accepted_contact=false;
};
} // namespace crash::cases::source_part_elastic
