// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"

namespace rigid_assembly_owner_test {
TEST(RigidAssemblyOwnerHost, CompactPreparedCoefficientsAndSourceNamespaces) {
  Fixture f;
  nd::RigidStorageLayout layout;
  ASSERT_EQ(nd::ForecastRigidStorage(f.binding,f.Config(),layout).status,Code::Ok);
  ASSERT_EQ(nd::ValidateRigidAssemblyOwner(f.binding,f.Kinematics(),f.im.data(),f.Dofs(),false).status,Code::Ok);
  std::unique_ptr<nd::RigidStorage> storage;
  ASSERT_EQ(nd::PrepareRigidStorage(f.binding,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),layout,storage).status,Code::Ok);
  ASSERT_EQ(storage->groups.size(),2);
  EXPECT_EQ(storage->info.part_group_count,1);
  EXPECT_EQ(storage->info.plain_source_instance_id,29);
  EXPECT_TRUE(storage->groups[0].dependent_coefficients);
  EXPECT_FALSE(storage->groups[1].dependent_coefficients);
  EXPECT_EQ(storage->properties[0].source_id,storage->properties[1].source_id);
  EXPECT_NE(storage->properties[0].source_kind,storage->properties[1].source_kind);
  EXPECT_EQ(storage->member_nodes[f.zero_mass],r::PartMemberNode);
  EXPECT_EQ(f.m[f.zero_mass],0);
  EXPECT_EQ(f.j[f.ordinary],0);
  EXPECT_EQ(f.present[f.ordinary],0);
  for (std::size_t k=0;k<storage->source_members.size();++k) {
    EXPECT_EQ(storage->source_members[k].mass_kg,f.binding.members()[k].mass_kg);
    EXPECT_EQ(storage->source_members[k].isotropic_inertia_kg_m2,f.binding.members()[k].isotropic_inertia_kg_m2);
  }
  std::array<double,36> state{};
  storage->InitializeState(state.data());
  const auto actual=r::ReadGroupState(state.data());
  EXPECT_EQ(actual.center.x,f.parts.roots()[0].value.raw.center.x);
  for (unsigned k=0;k<9;++k) EXPECT_EQ(actual.principal_axes.v[k],f.parts.roots()[0].value.principal.axes.v[k]);
  EXPECT_EQ(storage->owned_host_bytes,layout.host_bytes);
  // An exact appended owner identity must not open old shell admission.
  namespace physical=fe::native_physical_coefficients;
  EXPECT_FALSE(physical::ValidScope(storage->info,f.m.size()));
  auto changed=storage->info;
  changed.plain_source_instance_id++;
  EXPECT_FALSE(physical::SameScope(changed,storage->info));
}
TEST(RigidAssemblyOwnerHost, LateSourceCoefficientAndRotationMismatchPreserveOutputAndRetry) {
  Fixture f;
  nd::RigidStorageLayout layout;
  ASSERT_EQ(nd::ForecastRigidStorage(f.binding,f.Config(),layout).status,Code::Ok);
  std::unique_ptr<nd::RigidStorage> output;
  const auto last=f.binding.members()[f.binding.members().size()-1].domain_node;
  const auto original=f.ij[last];
  f.ij[last]=std::nextafter(original,INFINITY);
  EXPECT_EQ(nd::PrepareRigidStorage(f.binding,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),layout,output).status,Code::InvalidInput);
  EXPECT_FALSE(output);
  f.ij[last]=original;
  const auto x=f.x[3*f.ordinary];
  f.x[3*f.ordinary]=std::nextafter(x,INFINITY);
  EXPECT_EQ(nd::ValidateRigidAssemblyOwner(f.binding,f.Kinematics(),f.im.data(),f.Dofs(),false).status,Code::InvalidInput);
  f.x[3*f.ordinary]=x;
  f.present[f.zero_mass]=0;
  EXPECT_EQ(nd::PrepareRigidStorage(f.binding,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),layout,output).status,Code::InvalidInput);
  f.present[f.zero_mass]=1;
  ASSERT_EQ(nd::PrepareRigidStorage(f.binding,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),layout,output).status,Code::Ok);
}
TEST(RigidAssemblyOwnerHost, ExplicitCapacityOverflowAndExactRetainedBudget) {
  Fixture f;
  nd::RigidStorageLayout layout;
  auto config=f.Config();
  config.rigid_limits=fe::NodalRigidOwnerLimits::Vehicle();
  EXPECT_EQ(nd::ForecastRigidStorage(f.binding,config,layout).status,Code::ResourceLimit);
  config=f.Config();
  ASSERT_EQ(nd::ForecastRigidStorage(f.binding,config,layout).status,Code::Ok);
  const auto bytes=layout.host_bytes;
  config.rigid_limits.max_host_bytes=bytes-1;
  EXPECT_EQ(nd::ForecastRigidStorage(f.binding,config,layout).status,Code::ResourceLimit);
  config.rigid_limits.max_host_bytes=bytes;
  EXPECT_EQ(nd::ForecastRigidStorage(f.binding,config,layout).status,Code::Ok);
  nd::RigidStorageLayout vehicle;
  EXPECT_TRUE(vehicle.Initialize(524288,779,12991,fe::NodalRigidOwnerLimits::VehicleAssembly(),sizeof(nd::RigidStorage)));
  EXPECT_TRUE(vehicle.Initialize(1024,1,1024,fe::NodalRigidOwnerLimits::VehicleAssembly(),sizeof(nd::RigidStorage)));
  EXPECT_FALSE(vehicle.Initialize(1025,1,1025,fe::NodalRigidOwnerLimits::VehicleAssembly(),sizeof(nd::RigidStorage)));
  EXPECT_FALSE(vehicle.Initialize(524288,779,12991,fe::NodalRigidOwnerLimits::Vehicle(),sizeof(nd::RigidStorage)));
  EXPECT_FALSE(vehicle.Initialize(SIZE_MAX,1,2,fe::NodalRigidOwnerLimits::VehicleAssembly(),sizeof(nd::RigidStorage)));
}
TEST(RigidAssemblyOwnerHost, CinCopiesExactZeroInversesAndRejectsUnrelatedSourceChanges) {
  Fixture f;
  f.DependentInverses(true);
  auto startup=f.Cin();
  nd::CinLayout layout;
  ASSERT_EQ(nd::ForecastCinStorage(startup,f.Config(),layout).status,Code::Ok);
  std::unique_ptr<nd::CinStorage> storage;
  ASSERT_EQ(nd::PrepareCinStorage(startup,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),nullptr,layout,storage,&f.binding).status,Code::Ok);
  std::vector<double> state(layout.state_values);
  storage->InitializeState(state.data(),startup,f.im.data(),f.Dofs());
  const auto n=f.m.size();
  EXPECT_EQ(std::memcmp(state.data()+2*n,f.im.data(),n*sizeof(double)),0);
  EXPECT_EQ(std::memcmp(state.data()+3*n,f.ij.data(),n*sizeof(double)),0);
  EXPECT_EQ(state[2*n+f.zero_mass],0);
  EXPECT_EQ(state[3*n+f.zero_mass],0);
  EXPECT_EQ(state[3*n+f.ordinary],0);
  const auto before=storage.get();
  const auto original=f.m.back();
  f.m.back()=std::nextafter(original,INFINITY);
  EXPECT_EQ(nd::PrepareCinStorage(startup,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),nullptr,layout,storage,&f.binding).status,Code::InvalidInput);
  EXPECT_EQ(storage.get(),before);
  f.m.back()=original;
  ASSERT_EQ(nd::PrepareCinStorage(startup,f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),nullptr,layout,storage,&f.binding).status,Code::Ok);
}
TEST(RigidAssemblyOwnerHost, CinRejectsActualPartMasterIntersection) {
  Fixture f(true);
  f.DependentInverses(true);
  const auto startup=f.Cin();
  nd::CinLayout layout;
  ASSERT_EQ(nd::ForecastCinStorage(startup,f.Config(),layout).status,Code::Ok);
  std::unique_ptr<nd::CinStorage> storage;
  const auto report=nd::PrepareCinStorage(startup,f.Config(),f.Kinematics(),f.im.data(),
    f.Dofs(),nullptr,layout,storage,&f.binding);
  EXPECT_EQ(report.status,Code::InvalidInput);
  EXPECT_STREQ(report.message,"CIN master/dependent intersects an actual rigid member");
  EXPECT_FALSE(storage);
}
} // namespace rigid_assembly_owner_test
