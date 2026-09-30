// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"

namespace qbat_resident_test {
namespace {
qbat_force_test::Fixture NativeFixture(const fe::ShellFormulationScope& source,std::size_t parent) {
  qbat_force_test::Fixture fixture;
  fixture.reference=source.binding->qbat_reference(parent);
  fixture.input=fixture.reference.input();
  EXPECT_TRUE(source.catalog->Parameters(fe::ShellBindingFamily::Qbat,parent,&fixture.material));
  fixture.failure=source.failure->parent(fe::ShellBindingFamily::Qbat,parent)->constant;
  return fixture;
}
void Kinetic(const Rig& rig,const Prepared& prepared) {
  long double translation=0,rotation=0;
  for(std::size_t node=0;node<rig.binding->node_count();++node) {
    long double speed=0,spin=0;
    for(unsigned axis=0;axis<3;++axis) {
      const long double v=prepared.endpoint.v[3*node+axis],w=prepared.endpoint.omega[3*node+axis];
      speed+=v*v;
      spin+=w*w;
    }
    translation+=.5L*rig.binding->nodes()[node].native.mass*speed;
    rotation+=.5L*rig.binding->nodes()[node].native.isotropic_inertia*spin;
  }
  EXPECT_NEAR(prepared.diagnostics.kinetic.translation,double(translation),1e-13+1e-12*std::abs(double(translation)));
  EXPECT_NEAR(prepared.diagnostics.kinetic.rotation,double(rotation),1e-13+1e-12*std::abs(double(rotation)));
}
}
TEST(QbatResidentCuda, NativeFourPointYieldUnloadAndCommonRetryWithAbsentQeph) {
  MidlayerSource source;
  ASSERT_FALSE(HasFailure());
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(),false,false,0x1p-12));
  const auto allocation=rig.qbat.allocations();
  const auto native_fixture=NativeFixture(source.Scope(),0);
  qbat_force_test::NativeState native(native_fixture.Virgin().data());
  bool yielded=false;
  for(unsigned step=0;step<64;++step) {
    const double rate=60*std::cos(2*3.14159265358979323846*(step+.5)/32);
    Prepared prepared;
    ASSERT_TRUE(rig.Prepare(prepared,rate));
    const auto interval=Interval(rig,prepared,0);
    auto native_trial=native;
    native_trial.Step(native_fixture,interval);
    ASSERT_FALSE(HasFailure());
    qbat_force_test::CompareNative(Restore(Element(native_fixture),prepared.qbat[0]),native_trial);
    ASSERT_FALSE(HasFailure())<<step;
    EXPECT_FALSE(prepared.diagnostics.has_qeph);
    EXPECT_TRUE(prepared.diagnostics.has_t3);
    EXPECT_TRUE(prepared.diagnostics.has_qbat);
    EXPECT_FALSE(prepared.diagnostics.qbat.accepted_force_assembled);
    Kinetic(rig,prepared);
    if(step==7) {
      Fields before,after;
      std::vector<qb::BatchResult> base,still;
      fe::ShellBatchDiagnostics diagnostics;
      ASSERT_TRUE(rig.Accepted(before,base,diagnostics));
      auto bad=prepared.diagnostics;
      bad.qbat.attempt+=1;
      const auto& d=prepared.diagnostics.qbat;
      EXPECT_NE(rig.publication.Commit(rig.owner,prepared.token,bad,
          {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true}).status,fe::ShellPublicationStatus::Success);
      ASSERT_TRUE(rig.Accepted(after,still,diagnostics));
      SameFields(before,after);
      Exact(base,still);
      Prepared retry;
      ASSERT_TRUE(rig.Prepare(retry,rate));
      Exact(prepared.qbat,retry.qbat);
      EXPECT_NE(retry.view.attempt,prepared.view.attempt);
      EXPECT_EQ(retry.view.base_time,prepared.view.base_time);
      prepared=std::move(retry);
    }
    ASSERT_TRUE(rig.Commit(prepared));
    native=native_trial;
    for(const auto& point:prepared.qbat[0].history.point) yielded|=point.material.plastic_strain>1e-4;
    EXPECT_EQ(rig.qbat.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(rig.qbat.allocations().device_allocations,1u);
  }
  EXPECT_TRUE(yielded);
}
TEST(QbatResidentCuda, QbatOnlyCoupledTranslationAndExactMissingFamilyAdmission) {
  MidlayerSource source(false);
  ASSERT_FALSE(HasFailure());
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(),true,true));
  fe::ShellBatchPublication extra;
  EXPECT_NE(extra.InitializeFormulations(rig.owner,{&rig.qeph,nullptr,&rig.qbat,nullptr}).status,
      fe::ShellPublicationStatus::Success);
  for(unsigned step=0;step<8;++step) {
    Prepared prepared;
    ASSERT_TRUE(rig.Prepare(prepared));
    EXPECT_FALSE(prepared.diagnostics.has_qeph);
    EXPECT_FALSE(prepared.diagnostics.has_t3);
    EXPECT_TRUE(prepared.diagnostics.qbat.accepted_force_assembled);
    Kinetic(rig,prepared);
    ASSERT_TRUE(rig.Commit(prepared));
  }
  fe::ShellBatchDiagnostics accepted;
  ASSERT_EQ(rig.publication.CopyAcceptedDiagnostics(rig.owner.accepted(),&accepted).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(accepted.qbat.epoch,8u);
  EXPECT_EQ(accepted.t3.epoch,0u);
  EXPECT_EQ(accepted.qeph.epoch,0u);
}
TEST(QbatResidentCuda, ThreeOriginalLayerRolesShareQ4AndT3NodesWithoutDuplicateMass) {
  Source source(true);
  ASSERT_FALSE(HasFailure());
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(),true,true));
  Prepared prepared;
  ASSERT_TRUE(rig.Prepare(prepared));
  EXPECT_TRUE(prepared.diagnostics.has_qeph);
  EXPECT_TRUE(prepared.diagnostics.has_t3);
  EXPECT_EQ(prepared.diagnostics.qbat.element_count,1u);
  Kinetic(rig,prepared);
  ASSERT_TRUE(rig.Commit(prepared));
  std::vector<fe::ShellBatchLayeredSection> sections(3);
  fe::t3::BatchDiagnostics diagnostics;
  ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(),sections.data(),sections.size(),&diagnostics).status,
      fe::t3::BatchStatus::Success);
  EXPECT_EQ(sections[2].law(),fe::ShellSectionLaw::Law44Nip1);
  EXPECT_NE(sections[2].one_point(),nullptr);
}
TEST(QbatResidentCuda, NativeRemovalKeepsFourPointCacheAndLaterZeroAssembly) {
  MidlayerSource source(false,1e-5);
  ASSERT_FALSE(HasFailure());
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(),false,false,0x1p-12));
  const auto fixture=NativeFixture(source.Scope(),0);
  qbat_force_test::NativeState native(fixture.Virgin().data());
  unsigned removed=0;
  bool saw_removed=false;
  for(unsigned step=0;step<24;++step) {
    Prepared prepared;
    ASSERT_TRUE(rig.Prepare(prepared,30));
    native.Step(fixture,Interval(rig,prepared,0));
    ASSERT_FALSE(HasFailure());
    qbat_force_test::CompareNative(Restore(Element(fixture),prepared.qbat[0]),native);
    ASSERT_FALSE(HasFailure());
    removed+=prepared.diagnostics.qbat.newly_removed_count;
    if(!prepared.qbat[0].history.element_active) {
      saw_removed=true;
      for(const auto& force:prepared.qbat[0].internal_force_n) EXPECT_EQ(force.x*force.x+force.y*force.y+force.z*force.z,0);
    }
    ASSERT_TRUE(rig.Commit(prepared));
  }
  EXPECT_TRUE(saw_removed);
  EXPECT_EQ(removed,1u);
  // The removed accepted cache still participates in an authenticated assembly,
  // retaining native M/J and adding exactly zero force at every local node.
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  ASSERT_TRUE(rig.Assemble(view));
  std::vector<double> force(3*source.binding.node_count());
  const double* components[]{view.forces.force_x,view.forces.force_y,view.forces.force_z};
  for(unsigned axis=0;axis<3;++axis) {
    ASSERT_EQ(cudaMemcpy(force.data()+axis*source.binding.node_count(),components[axis],
        source.binding.node_count()*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  }
  for(double x:force) EXPECT_EQ(x,0);
  rig.Discard();
}
TEST(QbatResidentCuda, QephAndQbatWithoutT3UseExactCompleteInventory) {
  qbat_catalog_test::Fixture fixture;
  for(auto& quad:fixture.geometry.q) quad.reference.placement=fe::ShellReferencePlacement::Centered;
  const fe::ShellFormulationCollectionInput geometry{{fixture.geometry.q.data(),nullptr,2,0,4},&fixture.geometry.b,1};
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(geometry).status,fe::ShellBindingStatus::Success);
  const fe::ShellPlasticityParentInput parents[]{fixture.parents[0],fixture.parents[2],fixture.parents[4]};
  auto input=fixture.Input();
  input.parents=parents;
  input.parent_count=3;
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,input).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellFailureParentInput declarations[3];
  for(unsigned i=0;i<3;++i) declarations[i]=qbat_catalog_test::Failure(parents[i]);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,declarations,3).status,fe::ShellPlasticityBindingStatus::Success);
  Rig rig;
  ASSERT_TRUE(rig.Initialize({&binding,&catalog,&failure},true,true));
  Prepared prepared;
  ASSERT_TRUE(rig.Prepare(prepared));
  EXPECT_TRUE(prepared.diagnostics.has_qeph);
  EXPECT_FALSE(prepared.diagnostics.has_t3);
  EXPECT_TRUE(prepared.diagnostics.has_qbat);
  Kinetic(rig,prepared);
  ASSERT_TRUE(rig.Commit(prepared));
}
} // namespace qbat_resident_test
