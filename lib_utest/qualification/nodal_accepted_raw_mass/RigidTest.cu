#include "Fixture.h"
#include "../rigid_assembly_owner/OwnerFixture.h"
#include "lib_src/math/ScalarBits.h"

namespace accepted_mass_test {
namespace rig = rigid_assembly_owner_test;
TEST_F(Cuda, DisjointRigidMembersRetainTheirOriginalMassThroughCinCommits) {
  rig::Fixture fixture;
  fe::FENodalState owner;
  ASSERT_EQ(rig::Initialize(owner,fixture,true).status,Status::Ok);
  ASSERT_GT(owner.rigid_groups().group_count,0u);
  const auto n=fixture.m.size();
  const auto allocations=owner.allocations();
  for (unsigned step=0;step<3;++step) {
    SCOPED_TRACE(step);
    const auto loads=rig::Loads(fixture,step);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Status::Ok);
    View view;
    ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&view).status,Status::Ok);
    const auto mass=Read(view);
    for (const auto& member:fixture.binding.members()) {
      SCOPED_TRACE(member.domain_node);
      EXPECT_TRUE(tl::math::SameScalarBits(mass[member.domain_node],member.mass_kg));
      EXPECT_TRUE(tl::math::SameScalarBits(mass[member.domain_node],fixture.m[member.domain_node]));
    }
    double* component[]{assembly.forces.force_x,assembly.forces.force_y,assembly.forces.force_z,
      assembly.forces.couple_x,assembly.forces.couple_y,assembly.forces.couple_z};
    type25_friction_test::Drain drain{assembly.stream};
    for (unsigned axis=0;axis<6;++axis)
      ASSERT_EQ(cudaMemcpyAsync(component[axis],loads.data()+axis*n,n*sizeof(double),
        cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
    fe::NodalCinAssemblyView cin;
    ASSERT_EQ(owner.BorrowCinAssembly(token,&cin).status,Status::Ok);
    ASSERT_EQ(cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,assembly.stream),cudaSuccess);
    ASSERT_EQ(owner.SealAssembly(token).status,Status::Ok);
    ASSERT_EQ(rig::Advance(owner,token,assembly,true).status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(rig::Commit(owner,token,assembly));
    EXPECT_EQ(owner.accepted().epoch,step+1);
    EXPECT_NE(owner.AuthenticateAcceptedRawMass(token,view).status,Status::Ok);
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
}
} // namespace accepted_mass_test
