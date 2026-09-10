// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupModel.h"
namespace tl::fea::rigid {
// Startup values only. These records grant no owner, time or publication identity.
// A point's J is the native scalar IN already assembled by its actual producers;
// it may be zero. ELEMENT_MASS contributes mass and no rotational increment.
struct AssemblyMassPoint { Vec3 position; double mass=0,inertia=0; };
struct AssemblyPrimary { Vec3 position; double mass=0,inertia=0; };
struct AssemblyBodyInput {
  AssemblyPrimary primary;
  const AssemblyMassPoint* part=nullptr;
  const AssemblyMassPoint* extra=nullptr;
  std::size_t part_count=0,extra_count=0;
  NodalRigidSourceUnits source_units{}; // Required; native 1e-30 mass guard converts to SI.
};
struct AssemblyMassLedger {
  double part_mass=0,extra_mass=0,native_member_inertia=0;
  double primary_mass=0,primary_inertia=0;
  std::size_t part_members=0,extra_members=0,primaries=0;
};
struct AssemblyRawBody {
  double mass=0;
  Vec3 center;
  tl::math::Matrix3 tensor; // Raw world tensor, before any principal correction.
  AssemblyMassLedger ledger;
};
struct AssemblyFinalBody {
  AssemblyRawBody raw;
  PrincipalFrame principal;
  Vec3 raw_principal_inertia;
  tl::math::Matrix3 effective_tensor;
  NodalRigidRegularizationLedger regularization;
};
enum class AssemblyValueStatus { Success,InvalidInput,ResourceLimit,NonfiniteResult };
inline constexpr std::size_t MaxAssemblyBodyMembers=16384;

// All coefficients and positions are SI. Positive primary parameters are explicit
// converted source regularizers. No fallback
// mass/J is generated here. Supported source branch: ICOG1, N2D0, >=2 part nodes,
// optional extra nodes with Iflag2, no imposed skew/inertia/SPC/sensor/merge yet.
// All output/input overlaps, nonfinite inputs and failures preserve output.
AssemblyValueStatus PrepareAssemblyRawBody(const AssemblyBodyInput&,AssemblyRawBody&) noexcept;
// Native Iflag2 body merge. Inputs must still be RAW: Ispher2 is applied only
// after every child has been merged. No eigen solve or member state is performed.
AssemblyValueStatus MergeAssemblyRawBodies(const AssemblyRawBody& parent,
    const AssemblyRawBody& child,AssemblyRawBody&) noexcept;
// Reuses the exact existing model's Eigen decomposition and Ispher2 correction.
// Source identities and owner admission remain the assembly model's separate job.
NodalRigidGroupReport FinalizeAssemblyRawBody(const AssemblyRawBody&,
    AssemblyFinalBody&) noexcept;
} // namespace tl::fea::rigid
