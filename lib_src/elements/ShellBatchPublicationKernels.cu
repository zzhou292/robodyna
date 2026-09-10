// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationStorage.h"
#include "ShellBatchFields.h"

namespace tl::fea::shell_publication_detail {
namespace {
using namespace shell_batch_fields;
__device__ bool Measure(const Model& model,DeviceNodalKinematicsView view,ShellBatchKinetic& out) {
  for(std::size_t n=0;n<model.node_count;++n) {
    const auto v=ReadVector(view.velocity_xyz,n),w=ReadVector(view.angular_velocity_xyz,n);
    if(!FiniteVector(v)||!FiniteVector(w)) return false;
    const double vv=Dot(v,v),ww=Dot(w,w);
    // Same native scalar kinetic arithmetic as standalone shell diagnostics.
    out.translation+=.5*model.mass[n]*vv;
    out.rotation+=.5*model.inertia[n]*ww;
    out.physical_isotropic+=.5*model.physical[n]*ww;
    out.added_isotropic+=.5*model.added[n]*ww;
  }
  return tl::math::Finite(out.translation)&&tl::math::Finite(out.rotation)&&
    tl::math::Finite(out.physical_isotropic)&&tl::math::Finite(out.added_isotropic);
}
__global__ void MeasurePrepared(Storage* storage,NodalPreparedView view) {
  Control next;
  if(!Measure(storage->model,view.base_kinematics,next.base)||
     !Measure(storage->model,view.kinematics,next.endpoint)) next.status=ShellPublicationStatus::NonfiniteResult;
  storage->control=next;
}
} // namespace
void LaunchMeasure(Storage* storage,NodalPreparedView view) { MeasurePrepared<<<1,1,0,view.stream>>>(storage,view); }
} // namespace tl::fea::shell_publication_detail
