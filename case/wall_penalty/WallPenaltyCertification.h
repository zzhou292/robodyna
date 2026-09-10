#pragma once
#include "lib_src/collision/NodalWallContact.h"
#include <cstddef>

namespace crash::cases::wall_penalty {
namespace contact=tlfea::contact;
struct PenaltyDesign {
    double area_floor=1e-5,design_penetration=.000375,penetration_cap=.0005,kinetic_budget_factor=1.10;
};
enum class InitialKineticMetric { Unspecified,NativePhysicalNodes,NativeNodesWithAggregateGroups };
struct InitialKineticInput {
    // The caller owns the physical scope and phase proof. This immutable
    // enclosure is an initial collocated energy, never a reconstructed sample.
    contact::Q4CertifiedIntegral enclosure;
    InitialKineticMetric metric=InitialKineticMetric::Unspecified;
};
struct AreaFloorCertificate { double minimum_lower=0; unsigned node=UINT32_MAX; };
struct PenaltyCertificate {
    InitialKineticInput initial_kinetic;
    double kinetic_budget_upper=0,stiffness_per_area=0,design_potential_lower=0;
    double minimum_nodal_area_lower=0;
    unsigned minimum_area_node=UINT32_MAX,kappa_upward_steps=0;
};
enum class PenaltyStatus { Ok,InvalidInput,CertificateFailure,StepLimit };
struct PenaltyReport {
    PenaltyStatus status=PenaltyStatus::InvalidInput;
    const char* message="Invalid wall penalty request";
    unsigned node=UINT32_MAX;
    explicit operator bool() const noexcept { return status==PenaltyStatus::Ok; }
};
bool ValidPenaltyDesign(const PenaltyDesign&) noexcept;
PenaltyReport CertifyAreaFloor(const contact::NodalWallWeights&,std::size_t global_node_count,
    double declared_floor,AreaFloorCertificate*);
// Complete unique-node coverage is mandatory. Directed arithmetic and the
// fixed bounded ULP closure preserve the original source-part calculation.
// This energy inequality is a penalty design, not a coupled timestep proof.
PenaltyReport CertifyPenalty(const contact::NodalWallWeights&,std::size_t global_node_count,
    const InitialKineticInput&,const PenaltyDesign&,PenaltyCertificate*);
// Local native-mass rate diagnostic only, including for a group-constrained
// owner. Failure preserves the caller's value; structural admission is separate.
PenaltyReport CheckContactStep(double dt,double native_stiffness_rate,double limit,double* upper);
} // namespace crash::cases::wall_penalty
