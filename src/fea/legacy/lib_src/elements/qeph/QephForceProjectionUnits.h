// SPDX-License-Identifier: AGPL-3.0-or-later
// Rate/force adjoint boundary for the unchanged native CZPROJN leaf.
#pragma once
#include "QephForceProjection.h"
#include "QephProjectionUnits.h"
namespace tl::fea::qeph::detail {
TL_QEPH_HD inline Status ProjectForcesInWorkingLength(const GeometryWork& geometry,
    const LocalForceWork& local,double length,Vec3 (&force)[4],Vec3 (&couple)[4]) {
  if(!ValidProjectionLength(length)||geometry.values.projection_metric.working_length_m!=length)
    return Status::kInvalidReference;
  if(length==1.) {
    // Preserve original force leaf and its caller's existing finite-result
    // checks; no extra arithmetic is inserted into the legacy physical path.
    ProjectForces(geometry,local,force,couple);return Status::kSuccess;
  }
  if(!projection_units::Matrices(geometry.values))return Status::kNonfiniteResult;
  auto work=geometry;auto working_force=local;
  if(!projection_units::Divide(work.values.effective_warpage,length))return Status::kNonfiniteResult;
  for(unsigned n=0;n<4;++n) {
    if(!projection_units::Divide(work.values.local_position[n].x,length)||
        !projection_units::Divide(work.values.local_position[n].y,length)||
        !Finite(working_force.force[n])||
        !projection_units::Divide(working_force.couple[n][0],length)||
        !projection_units::Divide(working_force.couple[n][1],length))return Status::kNonfiniteResult;
  }
  // Force unit stays N; torque is N*working-length. These are numerical
  // projection operands only: material/history work remains in SI throughout.
  Vec3 next_force[4]{},next_couple[4]{};
  ProjectForces(work,working_force,next_force,next_couple);
  for(unsigned n=0;n<4;++n)
    if(!Finite(next_force[n])||!projection_units::Multiply(next_couple[n],length))
      return Status::kNonfiniteResult;
  for(unsigned n=0;n<4;++n){force[n]=next_force[n];couple[n]=next_couple[n];}
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail
