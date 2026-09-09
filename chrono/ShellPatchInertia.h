#pragma once

#include "ShellPatchAudit.h"

namespace crash::reference {

// HOST reference screen for exactly the existing six-node/two-Q4 patch. The
// inputs are actual TL row-sum ShellMass contributions, not inferred box
// inertias. All six nodes must occur; each element must name four distinct
// nodes. No constraints remove their positive mass from this physical ledger.
// Sharp seams with incompatible physical directors are outside this scope.
struct ShellPatchInertia {
    std::array<tl::fea::reissner::ShellNodalMass,kCouponNodes> original{};
    std::array<double,kCouponElements> element_area{};
    std::array<double,kCouponNodes> added_tangential_inertia{};
    std::array<double,kCouponNodes> added_drilling_inertia{};
    patch_audit::PatchNodalMass counterfactual;
    bool prepared=false;
};

// Each contribution adds DeltaJ_ei = m_ei * A_e / 12, where A_e is the
// WHOLE element area (sum of its four row-sum areas). This declared native-Q4
// area-inertia counterfactual is numerical, not physical director inertia or
// an adoption/qualification of its donor formulation. The explicitly chosen
// centered QEPH source branch INER_9_12=0 (FAC=12, ZSHIFT=0) is the reference:
// OpenRadioss a62b27e6baa555d222a580d6218867d0be4d70b5, starter/source/
// elements/shell/coque/cinmas.F:851-874,925-932,1381-1412; source closure and
// remaining mapping limits are in planning/OPENRADIOSS_SHELL_EXECUTION_TRACE.md.
// This operation reuses TL row-sum masses; it does not replace their producer
// with the native uniform-quarter distribution. Only the existing
// kEqualPhysicalTangential drilling policy is admitted: baseline total J*I
// has scalar J, not 2J. The counterfactual scalar is J + sum(DeltaJ_ei).
// Geometry/setup authenticity stays with the producer. All fields are staged;
// invalid, unused-node, overflow or unrepresentable-positive contributions
// leave output unchanged. Success clears diagnostic. No pointers are retained.
ElasticCouponStatus AssembleShellPatchInertia(
    const std::array<tl::fea::reissner::ShellMass,kCouponElements>& element_mass,
    const std::array<std::array<std::size_t,4>,kCouponElements>& connectivity,
    ShellPatchInertia& output,std::string& diagnostic);

struct ShellPatchKineticEnergy {
    double translation=0;
    double physical_rotation=0;
    double original_artificial_drilling=0;
    double added_tangential=0;
    double added_drilling=0;
};

// One assembled-node reduction, never added to a second element-wise ledger.
// Directors are physical unit WORLD directors including frame offsets; one
// common director per shared node is the declared patch convention. v/omega
// are WORLD values. No normalization, inertia policy or dynamics is applied.
// The original and added terms remain separate even though their sum is
// isotropic. A malformed/mutated report or late invalid kinematics preserves
// every output field; prepared is a default-data guard, not authentication.
ElasticCouponStatus ComputeShellPatchKineticEnergy(
    const ShellPatchInertia& inertia,
    const std::array<tl::fea::reissner::Vec3,kCouponNodes>& director,
    const std::array<tl::fea::reissner::Vec3,kCouponNodes>& velocity,
    const std::array<tl::fea::reissner::Vec3,kCouponNodes>& angular_velocity,
    ShellPatchKineticEnergy& output,std::string& diagnostic);

}  // namespace crash::reference
