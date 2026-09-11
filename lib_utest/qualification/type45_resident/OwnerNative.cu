// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type45_resident_test {
joint::Interval Interval(const joint::Joint& row,const fe::NodalPreparedView& view,double dt) {
  joint::Interval interval;
  interval.base_time_s=view.base_time;interval.dt_s=dt;interval.sample_index=view.kinematics.base_epoch+1;
  for(unsigned e=0;e<2;++e) {
    double x[3]{},w[3]{};
    EXPECT_EQ(cudaMemcpyAsync(x,view.kinematics.position_xyz+3*row.domain_nodes[e],sizeof(x),
        cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(w,view.kinematics.angular_velocity_xyz+3*row.domain_nodes[e],sizeof(w),
        cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
    interval.position_m[e]={x[0],x[1],x[2]};interval.angular_velocity_rad_s[e]={w[0],w[1],w[2]};
  }
  return interval;
}
std::vector<type45_test::NativeOracle> Native(Rig& rig,const fe::NodalTrialToken& token,const fe::NodalPreparedView& view) {
  std::vector<fe::NodalCinPhysicalMain> mains(rig.model.rigid_binding()->groups().size());
  fe::NodalCinPhysicalMainStamp receipt;
  EXPECT_TRUE(Good(rig.physical.owner.CopyPreparedCinPhysicalMains(token,{mains.data(),mains.size()},&receipt)));
  EXPECT_EQ(receipt.attempt,view.attempt);
  std::vector<type45_test::NativeOracle> result;
  for(const auto& row:rig.model.joints()) {
    type45_test::Fixture fixture(row.property.kind);
    fixture.property=row.property;fixture.geometry=row.geometry;fixture.context.target_dt_s=receipt.owner_fixed_dt;
    for(unsigned e=0;e<2;++e) {
      fixture.damping[e]=row.damping[e];const auto& main=mains[row.body_groups[e]];
      fixture.context.main[e]={joint::EndpointRole::RigidMember,main.source_group_id,main.center_m,
        main.mass_kg,main.minimum_principal_inertia_kg_m2,main.translational_stiffness_n_per_m,main.rotational_stiffness_nm};
    }
    int status=-1;result.emplace_back(fixture,status);EXPECT_EQ(status,0);
    // Independent native startup uses the actual query packet, never the GPU
    // reference or a copied automatic K. The comparison checks that reference.
    type45_test::CompareReference(fixture,fixture.Prepare(),result.back());
  }
  return result;
}
void ComparePrepared(Rig& rig,const fe::NodalPreparedView& view,const joint::BatchDiagnostics& d,
    std::vector<type45_test::NativeOracle>& native) {
  std::array<joint::Result,3> results;
  ASSERT_TRUE(Good(rig.joints.CopyPreparedResults(d,{results.data(),results.size()})));
  for(std::size_t j=0;j<results.size();++j) {
    SCOPED_TRACE(j);
    const auto actual=joint::BatchQualificationPeer::Staged(rig.joints,j);
    ASSERT_TRUE(actual.history.reference().Matches(native[j].compared_reference));
    type45_native::Step expected;
    ASSERT_TRUE(native[j].Step(Interval(rig.model.joints()[j],view,rig.physical.owner.accepted().fixed_dt),expected));
    type45_test::CompareStep(actual,native[j],expected);
    for(unsigned dof=0;dof<6;++dof) if(!joint::detail::Blocked(rig.model.joints()[j].property.kind,dof))
      EXPECT_EQ(joint::detail::Get(dof<3?actual.history.values().local_force_n:actual.history.values().local_couple_nm,
          dof<3?dof:dof-3),0);
    EXPECT_TRUE(results[j].automatic_stiffness_initialized);
    EXPECT_EQ(results[j].stamp.sample_index,view.kinematics.base_epoch+1);
  }
}
} // namespace type45_resident_test
