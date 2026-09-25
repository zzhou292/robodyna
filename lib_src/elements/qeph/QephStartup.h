// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// CDERII and selected typed-placement CINMAS expressions; see the pinned source map
// in lib_utest/qualification/qeph/source-manifest.json and adjacent LICENSE.md.
#pragma once
#include "lib_src/elements/ShellPlacementCoefficients.h"
#include "QephStartupFrame.h"

#if defined(__CUDACC__)
#define TL_QEPH_STARTUP_HD __host__ __device__
#else
#define TL_QEPH_STARTUP_HD
#endif

namespace tl::fea::qeph {
// Startup only, fixed IREP0 QEPH with TYPE1 placement. The native frame determinant
// exceeds 1e-20 m^2 plus the declared 64epsilon boundary band; normalized corner
// turns exceed128epsilon. Material, projected coordinates, mass and all inertia
// partitions must remain finite, and every mass/inertia is strictly positive.
// This limited binary64 domain is not a full geometry certificate. No state,
// force, batch, time integration or mass-scaling policy is introduced here.
// Failure preserves all output bytes; successful initialization may replace it.
TL_QEPH_STARTUP_HD inline Status InitializeReference(const ReferenceInput& input,ReferenceData& output) {
  if(!detail::ValidProjectionLength(input.projection_working_length_m)||
      !ValidShellReferencePlacement(input.placement)||!detail::Positive(input.density)||
      !detail::Positive(input.young_modulus)||
      !detail::Positive(input.thickness)||!tl::math::Finite(input.poisson_ratio)||
      input.poisson_ratio<0||input.poisson_ratio>=.5) return Status::kInvalidInput;
  for (unsigned i=0;i<4;++i) {
    if (!detail::Finite(input.position[i])) return Status::kInvalidInput;
    for (unsigned j=0;j<i;++j)
      if (input.node_ids[i]==input.node_ids[j]) return Status::kInvalidInput;
  }
  ReferenceData candidate;
  candidate.input=input;
  const auto frame_status=detail::StartupFrame(input.position,candidate.frame,candidate.area);
  if (frame_status!=Status::kSuccess) return frame_status;
  if (!detail::ConvexProjection(input.position,candidate.frame)) return Status::kUnsupportedGeometry;
  // Complete selected CDERII: node0-relative world differences are projected
  // onto the two native axes. The reported local z remains zero for warped Q4.
  const auto& f=candidate.frame;
  for (unsigned i=1;i<4;++i) {
    const double dx=input.position[i].x-input.position[0].x;
    const double dy=input.position[i].y-input.position[0].y;
    const double dz=input.position[i].z-input.position[0].z;
    candidate.local_position[i].x=f.v[0]*dx+f.v[3]*dy+f.v[6]*dz;
    candidate.local_position[i].y=f.v[1]*dx+f.v[4]*dy+f.v[7]*dz;
    if (!detail::Finite(candidate.local_position[i])) return Status::kNonfiniteResult;
  }
  const double px1=.5*(candidate.local_position[1].y-candidate.local_position[3].y);
  const double px2=.5*candidate.local_position[2].y;
  const double py1=-.5*(candidate.local_position[1].x-candidate.local_position[3].x);
  const double py2=-.5*candidate.local_position[2].x;
  const double px[4]{px1,px2,-px1,-px2},py[4]{py1,py2,-py1,-py2};
  // Exact native total expression and independent diagnostic partitions. Do
  // not sum partitions to replace the total or inject Reissner drilling mass.
  const double mass=input.density*input.thickness*candidate.area*.25;
  const double shift=NativeShellShift(input.placement);
  const double physical=mass*input.thickness*input.thickness*
      (input.placement==ShellReferencePlacement::Centered ? 1./12 : 1./12+shift*shift);
  const double added=mass*(candidate.area/12);
  const double total=NativeQephPlacementInertia(mass,candidate.area,input.thickness,
      input.placement);
  if (!detail::Positive(mass)||!detail::Positive(physical)||
      !detail::Positive(added)||!detail::Positive(total)) return Status::kNonfiniteResult;
  for (unsigned i=0;i<4;++i) {
    if (!tl::math::Finite(px[i])||!tl::math::Finite(py[i])) return Status::kNonfiniteResult;
    candidate.derivative_x[i]=px[i]; candidate.derivative_y[i]=py[i];
    candidate.nodal_mass[i]=mass; candidate.physical_inertia[i]=physical;
    candidate.added_inertia[i]=added; candidate.isotropic_inertia[i]=total;
  }
  candidate.prepared=true;
  output=candidate;
  return Status::kSuccess;
}
}  // namespace tl::fea::qeph
#undef TL_QEPH_STARTUP_HD
