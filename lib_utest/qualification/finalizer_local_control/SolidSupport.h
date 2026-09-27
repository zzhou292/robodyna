// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/solid_measurement_operands/TestSupport.h"
#include "Control2733SolidFinalize.h"
#include "CurrentSolidFinalize.h"
namespace final_control_test::solid {
namespace f=solid_operands_test;
namespace s=tl::fea::solids;
namespace b=s::batch_detail;
namespace fe=tl::fea;
inline s::BatchDiagnostics Seed(bool valid,double incoming) {
  s::BatchDiagnostics out;out.source_instance_id=61;out.owner_id=67;out.configuration_id=71;out.qualification_id=73;
  out.base_epoch=79;out.attempt=83;out.base_time=-0.;out.velocity_time=.125;
  out.base_velocity_time=-.125;out.kick_dt=-.25;out.phase=s::BatchPhase::Prepared;
  out.valid=valid;out.has_completed_interval=true;out.accepted_force_assembled=true;
  for(unsigned i=0;i<5;++i) {
    out.parent_count[i]=91+i;out.native_internal_work_increment_j[i]=incoming;
    out.physical_hourglass_work_increment_j[i]=(i%2)?-.5:.25;
  }
  out.plastic_work_increment_j=3;out.internal_kick_work_j=incoming;
  out.internal_drift_work_j=-incoming;out.minimum_native_dt_s=-17;return out;
}
inline b::Control Poison() {
  b::Control out{};out.status=s::BatchStatus::NonfiniteResult;out.family=s::Family::Solid18Law90;
  out.parent=101;out.node=103;out.element_status=107;out.diagnostics=Seed(true,109);return out;
}
inline void Compare(f::HostRig& rig,const s::BatchDiagnostics& identity,bool initial,bool operands,
    fe::NodalPreparedView view={}) {
  // Reuse the existing source coordinates for a stationary complete nodal view.
  // Candidate finalization always has a nonnull view, unlike initial binding.
  std::vector<double> positions,velocity;
  if(!initial && !view.kinematics.position_xyz) {
    const auto nodes=rig.model.domain()->nodes();positions.resize(3*nodes.size());
    velocity.assign(positions.size(),0.);
    for(std::size_t n=0;n<nodes.size();++n) {
      positions[3*n]=nodes[n].position.x;positions[3*n+1]=nodes[n].position.y;positions[3*n+2]=nodes[n].position.z;
    }
    view.base_kinematics.position_xyz=view.kinematics.position_xyz=positions.data();
    view.base_kinematics.velocity_xyz=view.kinematics.velocity_xyz=velocity.data();
  }
  const auto poison=Poison();rig.state.control=poison;
  b::control2733::Finalize(&rig.state,0,1,view,identity,initial,operands);
  const auto expected=rig.state.control;rig.state.control=poison;
  b::control_test::CurrentFinalize(&rig.state,0,1,view,identity,initial,operands);
  f::SameControl(rig.state.control,expected);
}
} // namespace final_control_test::solid
