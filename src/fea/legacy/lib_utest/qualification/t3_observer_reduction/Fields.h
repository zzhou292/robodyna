// SPDX-License-Identifier: MIT
#pragma once
#include "lib_utest/qualification/t3_mapped_gather/Fixture.h"
#include "lib_src/elements/t3/mapped/ObserverValues.h"
#include "SerialDiagnostics.h"

namespace t3_observer_test {
using namespace t3_gather_test;
struct Fields {
  double position[3*Nodes]{},velocity[3*Nodes]{},omega[3*Nodes]{},orientation[4*Nodes]{};
  fe::NodalPreparedView View(Inputs& base,unsigned epoch) {
    fe::NodalPreparedView v;
    v.base_kinematics={base.position,base.velocity,base.omega,Nodes,epoch,base.orientation};
    v.kinematics={position,velocity,omega,Nodes,epoch,orientation};
    v.base_time=epoch*1e-6;v.proposed_time=(epoch+1)*1e-6;
    v.kick_dt=epoch?1e-6:.5e-6;
    return v;
  }
};
inline q::BatchDiagnostics Identity(unsigned epoch,bool assembled=true) {
  q::BatchDiagnostics d;
  d.owner_id=7;d.configuration_id=11;d.qualification_id=13;
  d.epoch=epoch+1;d.base_epoch=epoch;d.attempt=17+epoch;
  d.time=(epoch+1)*1e-6;d.base_time=epoch*1e-6;
  d.velocity_time=(epoch+.5)*1e-6;d.base_velocity_time=epoch?(epoch-.5)*1e-6:0;
  d.kick_dt=epoch?1e-6:.5e-6;d.phase=q::BatchPhase::Prepared;
  d.has_completed_interval=true;d.accepted_force_assembled=assembled;
  d.kinetic_available=false;d.usage=q::BatchUsage::CoupledForces;
  return d;
}
struct BaseFixture : t3_gather_test::Fixture {
  Fields fields;
  void Reset(unsigned epoch,unsigned mask=0) {
    Prepare(epoch+1);
    for(unsigned p=0;p<Parents;++p)host->slab[1].element[p]=host->slab[0].element[p];
    Prepare(epoch);
    host->model.config.owner.fixed_dt=1e-6;
    for(unsigned n=0;n<Nodes;++n) {
      fields.orientation[4*n]=1;
      for(unsigned a=0;a<3;++a) {
        const unsigned j=3*n+a;
        input.velocity[j]=.125*(1+j);input.omega[j]=-.0625*j;
        fields.position[j]=input.position[j]+1e-6*(1+j);
        fields.velocity[j]=input.velocity[j]+.03125*(1+j);
        fields.omega[j]=input.omega[j]-.015625*(1+j);
      }
    }
    for(unsigned p=0;p<Parents;++p) {
      host->candidate_status[p]=q::Status::kSuccess;
      if(p==3)continue;
      auto& next=host->slab[1].element[p];
      auto values=next.proposed_history.data();
      const double work=p==0?0x1p54:p==1?1:-0x1p54;
      values.internal_work[0]=work;values.internal_work[1]=-work;
      values.strain_curvature[0]=-.01*(p+1);values.strain_curvature[5]=2*(p+1);
      values.active=mask&(1u<<p)?0:1;
      EXPECT_EQ(q::PrepareFailurePrescribedHistory(host->model.element[p].reference,values,
          {(epoch+1)*1e-6,epoch+1},next.proposed_history),q::Status::kSuccess);
      next.diagnostics.internal_work_increment[0]=work;
      next.diagnostics.internal_work_increment[1]=-.5*work;
      next.diagnostics.unscaled_element_dt=1e-4/(p+1);
      if(!values.active) {
        for(auto& f:next.internal_force)f={-0.,0.,-0.};
        for(auto& c:next.internal_couple)c={0.,-0.,0.};
      }
      EXPECT_TRUE(m::ValidResult(host->model.element[p].reference,next,(epoch+1)*1e-6,epoch+1,false));
    }
  }
};
inline void SameControl(const b::Control& a,const b::Control& z) {
  EXPECT_EQ(a.status,z.status);EXPECT_EQ(a.element_status,z.element_status);
  EXPECT_EQ(a.element,z.element);EXPECT_EQ(a.node,z.node);
  const auto& x=a.diagnostics;const auto& y=z.diagnostics;
#define CHECK_FIELD(name) EXPECT_EQ(x.name,y.name)<<#name
  CHECK_FIELD(owner_id);CHECK_FIELD(configuration_id);CHECK_FIELD(qualification_id);
  CHECK_FIELD(epoch);CHECK_FIELD(base_epoch);CHECK_FIELD(attempt);CHECK_FIELD(phase);
  CHECK_FIELD(valid);CHECK_FIELD(has_completed_interval);CHECK_FIELD(accepted_force_assembled);
  CHECK_FIELD(kinetic_available);CHECK_FIELD(usage);
#undef CHECK_FIELD
#define CHECK_BITS(name) EXPECT_EQ(Bits(x.name),Bits(y.name))<<#name
  CHECK_BITS(time);CHECK_BITS(base_time);CHECK_BITS(velocity_time);CHECK_BITS(base_velocity_time);CHECK_BITS(kick_dt);
  CHECK_BITS(kinetic_translation);CHECK_BITS(kinetic_rotation);CHECK_BITS(kinetic_physical_isotropic);CHECK_BITS(kinetic_added_isotropic);
  CHECK_BITS(internal_work[0]);CHECK_BITS(internal_work[1]);CHECK_BITS(internal_work_increment[0]);CHECK_BITS(internal_work_increment[1]);
  CHECK_BITS(minimum_area_ratio);CHECK_BITS(minimum_thickness_ratio);CHECK_BITS(minimum_native_dt);
  CHECK_BITS(maximum_displacement);CHECK_BITS(maximum_absolute_strain);CHECK_BITS(maximum_thickness_curvature);
  CHECK_BITS(internal_kick_work);CHECK_BITS(internal_drift_work);
#undef CHECK_BITS
}
} // namespace t3_observer_test
