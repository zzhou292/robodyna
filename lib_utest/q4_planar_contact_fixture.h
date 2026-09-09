#pragma once

#include "lib_src/collision/Q4PlanarContact.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_utest/q4_planar_geometry_fixture.h"

#include <gtest/gtest.h>
#include <array>
#include <cstring>

namespace q4_contact_batch_test {
namespace sc=tlfea::contact;
namespace fea=tl::fea;
using Code=sc::Q4PlanarContactStatus;
constexpr std::uint64_t ContactTestQualification=0x4334513450524f42ULL;

struct Rig : q4_planar_test::Pair {
  std::array<double,18> position{};
  std::array<double,18> omega{};
  std::array<double,24> rotation{};
  std::array<double,6> inverse_inertia{};
  std::array<std::uint8_t,6> rotation_fixed{{1,1,1,1,1,1}};
  fea::NodalReport Initialize(fea::FENodalState& owner,double h=1e-4) {
    for (unsigned n=0;n<6;++n) {
      for (unsigned axis=0;axis<3;++axis) position[3*n+axis]=x[n+6*axis];
      rotation[4*n]=1;
    }
    fea::NodalStateConfig config; config.node_count=6; config.fixed_dt=h;
    return owner.Initialize(config,{position.data(),v.data(),omega.data(),6,rotation.data()},inverse.data(),
        {fixed.data(),rotation_fixed.data(),inverse_inertia.data()});
  }
  sc::Q4FixedYZMassView mass_for(const fea::FENodalState& owner) const {
    return {inverse.data(),fixed.data(),6,owner.accepted().epoch};
  }
  sc::Q4PlanarContactConfig config(const fea::FENodalState& owner) const {
    sc::Q4PlanarContactConfig value; value.owner=owner.accepted();
    value.configuration_id=701; value.wall_binding_id=q4_planar_test::LargeId+801;
    value.stiffness_per_area=16; value.maximum_penetration=.1;
    value.integration={sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationDepth,sc::MaxQ4IntegrationVisits,1e-5,1e-7};
    return value;
  }
};
struct Snapshot {
  std::array<double,18> x{},v{},omega{},reaction{},couple{};
  std::array<double,24> q{};
  fea::NodalStamp stamp;
};
inline Snapshot Read(fea::FENodalState& owner) {
  Snapshot output;
  EXPECT_EQ(owner.CopyAccepted({output.x.data(),output.v.data(),6,output.q.data(),output.omega.data(),
                               output.reaction.data(),output.couple.data()},&output.stamp).status,fea::NodalStatus::Ok);
  return output;
}
inline void SameState(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.q,b.q); EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple);
  EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch,b.stamp.epoch); EXPECT_EQ(a.stamp.time,b.stamp.time);
  EXPECT_EQ(a.stamp.reactions_valid,b.stamp.reactions_valid);
  EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch); EXPECT_EQ(a.stamp.reaction_time,b.stamp.reaction_time);
}
inline std::array<double,36> Forces(const fea::NodalAssemblyView& view) {
  std::array<double,36> values{};
  const double* components[6]={view.forces.force_x,view.forces.force_y,view.forces.force_z,
                                view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for (unsigned axis=0;axis<6;++axis)
    EXPECT_EQ(cudaMemcpyAsync(values.data()+6*axis,components[axis],6*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess); return values;
}
template<class T> inline void Unchanged(const T& output,const T& before) {
  EXPECT_EQ(std::memcmp(&output,&before,sizeof(T)),0);
}
inline fea::NodalStepAdmission Admission(const fea::NodalAssemblyView& view,const sc::Q4PlanarContact& batch,double h) {
  fea::NodalStepAdmission value;
  value.owner_id=view.owner_id; value.base_epoch=view.accepted.base_epoch; value.attempt=view.attempt;
  value.maximum_dt=h; value.maximum_rotation_increment=.1;
  value.kind=fea::NodalStepAdmissionKind::RestrictedElasticTrajectory;
  value.qualification_id=ContactTestQualification;
  value.stiffness_rate_envelope=batch.stiffness_rate_bound();
  return value;
}
}  // namespace q4_contact_batch_test
