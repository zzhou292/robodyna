// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MovingSceneRig.h"
#include "Reference.h"
#include "../radioss_type25_local_geometry/Assertions.h"
#include <gtest/gtest.h>
#include <cstring>
namespace native_runtime_test {
namespace {
void Number(double actual,double expected,double absolute,double relative=2e-8) {
  ASSERT_TRUE(std::isfinite(actual));ASSERT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(actual,expected,absolute+relative*std::abs(expected));
}
void History(const n::NativeContactRow& actual,const n::NativeContactRow& expected) {
  for(unsigned i=0;i<4;++i)ASSERT_EQ(actual.irtlm[i],expected.irtlm[i]);
  const auto& x=actual.history.normal;const auto& y=expected.history.normal;
  Number(x.previous_penetration,y.previous_penetration,2e-8);Number(x.staged_penetration,y.staged_penetration,2e-8);
  Number(x.previous_stiffness,y.previous_stiffness,2e-3);Number(x.staged_stiffness,y.staged_stiffness,2e-3);
  Number(x.damping_half_force,y.damping_half_force,2e-5);
  const double a[]{actual.history.previous_force.x,actual.history.previous_force.y,actual.history.previous_force.z,
    actual.history.staged_force.x,actual.history.staged_force.y,actual.history.staged_force.z};
  const double b[]{expected.history.previous_force.x,expected.history.previous_force.y,expected.history.previous_force.z,
    expected.history.staged_force.x,expected.history.staged_force.y,expected.history.staged_force.z};
  for(unsigned i=0;i<6;++i)Number(a[i],b[i],2e-5);
  Number(actual.penetration_auxiliary,expected.penetration_auxiliary,2e-8);
  Number(actual.penetration_offset,expected.penetration_offset,2e-8);
  for(unsigned i=0;i<2;++i) {
    if(std::abs(expected.selection_metric[i])==1e20)EXPECT_EQ(actual.selection_metric[i],expected.selection_metric[i]);
    else Number(actual.selection_metric[i],expected.selection_metric[i],2e-8);
  }
}
void Motion(const State& actual,const ExpectedFrame& expected) {
  ASSERT_EQ(actual.stamp.epoch,expected.epoch);Number(actual.stamp.time,expected.time,1e-16,0);
  for(unsigned i=0;i<54;++i){SCOPED_TRACE(i);Number(actual.x[i]/.001,expected.position[i],2e-8);
    Number(actual.v[i]/.001,expected.velocity[i],2e-5);}
}
void PacketOrder(const n::runtime_qualification::Observation& actual,const ExpectedFrame& expected) {
  ASSERT_EQ(actual.force_base_epoch,expected.epoch);std::vector<int> secondary,main;
  for(const auto& occurrence:actual.occurrences)if(occurrence.secondary>0){secondary.push_back(occurrence.secondary);main.push_back(occurrence.selected.local_main);}
  ASSERT_EQ(actual.cohort_ends.size(),expected.geometry_packets.size());std::size_t first=0;
  for(std::size_t i=0;i<actual.cohort_ends.size();++i) {
    const auto last=actual.cohort_ends[i];ASSERT_LE(last,secondary.size());
    EXPECT_EQ(std::vector<int>(secondary.begin()+first,secondary.begin()+last),expected.geometry_packets[i].secondary);
    EXPECT_EQ(std::vector<int>(main.begin()+first,main.begin()+last),expected.geometry_packets[i].main);first=last;
  }
  EXPECT_EQ(first,secondary.size());
}
void Exact(const State& a,const State& b) {
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  const auto same=[](const auto& x,const auto& y){EXPECT_EQ(std::memcmp(x.data(),y.data(),x.size()*sizeof(x[0])),0);};
  same(a.x,b.x);same(a.v,b.v);same(a.q,b.q);same(a.omega,b.omega);same(a.mass,b.mass);same(a.inertia,b.inertia);
  same(a.reaction,b.reaction);same(a.couple,b.couple);EXPECT_EQ(a.initial_contact,b.initial_contact);
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.contact.stamp,b.contact.stamp));
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.contact.force_base_stamp,b.contact.force_base_stamp));
  EXPECT_EQ(a.contact.generation,b.contact.generation);EXPECT_EQ(a.contact.selectors.history,b.contact.selectors.history);
  EXPECT_EQ(a.contact.selectors.reference,b.contact.selectors.reference);EXPECT_EQ(a.contact.selectors.reference_generation,b.contact.selectors.reference_generation);
  for(unsigned i=0;i<18;++i)type25_geometry_test::Same(a.history[i],b.history[i]);
}
}
TEST(NativeType25MovingSceneCuda,ActualOwnerInitialMassInertiaAndReadbackAdmission) {
  Rig rig;ASSERT_NO_THROW(rig.Initialize());const auto state=rig.Read();
  EXPECT_FALSE(state.contact.force_phase_available);EXPECT_FALSE(state.contact.selectors.has_reference);
  for(unsigned i=0;i<18;++i) {SCOPED_TRACE(i);Number(state.mass[i],observed::ObservedMass[i]*1000,0,128*std::numeric_limits<double>::epsilon());
    Number(state.inertia[i],observed::ObservedInertia[i]*.001,0,128*std::numeric_limits<double>::epsilon());
    EXPECT_GT(state.mass[i],0);EXPECT_GT(state.inertia[i],0);if(i<9){EXPECT_EQ(rig.fixture.inverse_mass[i],0);EXPECT_EQ(rig.fixture.inverse_inertia[i],0);}}
  std::array<int,18> flags{};fe::NativeContactPublicationSnapshot snapshot;
  EXPECT_EQ(rig.contact.CopyAccepted({reinterpret_cast<n::NativeGeometryHistory*>(&rig.contact),flags.data(),18},&snapshot).status,n::TransactionStatus::InvalidInput);
  EXPECT_EQ(rig.contact.CopyAccepted({nullptr,flags.data(),18},&snapshot).status,n::TransactionStatus::InvalidInput);
  EXPECT_EQ(rig.contact.CopyAccepted({const_cast<n::NativeGeometryHistory*>(state.history.data()),flags.data(),17},&snapshot).status,n::TransactionStatus::InvalidInput);
  Exact(state,rig.Read());
}
TEST(NativeType25MovingSceneCuda,FullCappedTrajectoryHistoryForceAndPacketsMatchExternalNative) {
  Rig rig;ASSERT_NO_THROW(rig.Initialize());ReferenceReader reference(TYPE25_NATIVE_REFERENCE_FILE);
  State accepted=rig.Read();std::array<bool,18> prior{};std::array<unsigned,18> episodes{};
  std::size_t active_steps=0,rebuilds=0;
  RecordProperty("coordinate_absolute_tolerance_native_mm","2e-8");
  RecordProperty("velocity_absolute_tolerance_native_mm_per_s","2e-5");
  RecordProperty("force_absolute_tolerance_native_n","2e-5");
  RecordProperty("stiffness_absolute_tolerance_native_n_per_mm","2e-3");
  RecordProperty("relative_tolerance","2e-8");
  for(std::uint64_t step=0;step<1000;++step) {
    SCOPED_TRACE(step);const auto expected=reference.Read(step);ASSERT_NO_FATAL_FAILURE(Motion(accepted,expected));
    Attempt attempt;ASSERT_NO_THROW(rig.BeginMaterials(attempt));
    n::runtime_qualification::Observation observation;
    EXPECT_FALSE(n::runtime_qualification::Access::Read(rig.contact,rig.owner,attempt.token,attempt.assembly,&observation));
    std::array<double,54> before{},after{};std::array<double,18> before_k{},after_k{};
    ASSERT_NO_THROW(rig.Force(attempt,before,before_k));ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,attempt.token,attempt.assembly)));
    ASSERT_TRUE(n::runtime_qualification::Access::Read(rig.contact,rig.owner,attempt.token,attempt.assembly,&observation));
    ASSERT_NO_FATAL_FAILURE(PacketOrder(observation,expected));ASSERT_NO_THROW(rig.Force(attempt,after,after_k));
    for(unsigned i=0;i<54;++i){SCOPED_TRACE(i);Number(after[i]-before[i],expected.outgoing_force[i]-expected.incoming_force[i],2e-5);}
    for(unsigned i=0;i<18;++i)Number((after_k[i]-before_k[i])/1000,expected.outgoing_stiffness[i]-expected.incoming_stiffness[i],2e-3);
    const auto diagnostics=rig.contact.last_diagnostics();active_steps+=diagnostics.active_forces!=0;rebuilds+=diagnostics.reference_rebuilt;
    ASSERT_NO_THROW(rig.Prepare(attempt));ASSERT_NO_THROW(Check(rig.Commit(attempt)));accepted=rig.Read();
    EXPECT_FALSE(n::runtime_qualification::Access::Read(rig.contact,rig.owner,attempt.token,attempt.assembly,&observation));
    ASSERT_TRUE(accepted.contact.force_phase_available);EXPECT_EQ(accepted.contact.force_base_stamp.epoch,step);
    EXPECT_EQ(accepted.contact.generation,step+1);
    for(unsigned row=0;row<18;++row){SCOPED_TRACE(row);EXPECT_EQ(accepted.history[row].secondary_source_id,observed::NodeIds[observed::SecondaryNodes[row]]);
      ASSERT_NO_FATAL_FAILURE(History(accepted.history[row].row,expected.rows[row]));EXPECT_EQ(accepted.initial_contact[row],expected.initial_contact[row]);
      const bool active=accepted.history[row].row.irtlm[0]!=0;if(active&&!prior[row])++episodes[row];prior[row]=active;}
    if(HasFailure())return;
  }
  const auto terminal=reference.Read(1000);ASSERT_NO_FATAL_FAILURE(Motion(accepted,terminal));reference.Finish();
  EXPECT_EQ(active_steps,35u);EXPECT_EQ(episodes[12],2u);EXPECT_GE(rebuilds,1u);
  for(const auto& history:accepted.history)EXPECT_EQ(history.row.irtlm[0],0);
  RecordProperty("accepted_physical_intervals","1000");RecordProperty("reference_rebuilds",std::to_string(rebuilds));
}
TEST(NativeType25MovingSceneCuda,ActiveForceCommonRejectionRetryPreservesEveryAcceptedField) {
  Rig rig;ASSERT_NO_THROW(rig.Initialize());for(unsigned i=0;i<468;++i)ASSERT_NO_THROW(rig.Step());
  const auto before=rig.Read();Attempt rejected;ASSERT_NO_THROW(rig.BeginMaterials(rejected));
  ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,rejected.token,rejected.assembly)));
  ASSERT_GT(rig.contact.last_diagnostics().active_forces,0u);ASSERT_NO_THROW(rig.Prepare(rejected));
  EXPECT_NE(rig.Commit(rejected,false).status,fe::ShellPublicationStatus::Success);Exact(before,rig.Read());
  n::runtime_qualification::Observation stale;
  EXPECT_FALSE(n::runtime_qualification::Access::Read(rig.contact,rig.owner,rejected.token,rejected.assembly,&stale));
  Attempt retry;ASSERT_NO_THROW(rig.BeginMaterials(retry));ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,retry.token,retry.assembly)));
  ASSERT_NO_THROW(rig.Prepare(retry));ASSERT_NO_THROW(Check(rig.Commit(retry)));const auto after=rig.Read();
  EXPECT_EQ(after.stamp.epoch,before.stamp.epoch+1);EXPECT_EQ(after.contact.force_base_stamp.epoch,before.stamp.epoch);
  ReferenceReader expected(TYPE25_NATIVE_REFERENCE_FILE);ExpectedFrame frame;for(unsigned i=0;i<=468;++i)frame=expected.Read(i);
  for(unsigned row=0;row<18;++row)ASSERT_NO_FATAL_FAILURE(History(after.history[row].row,frame.rows[row]));
}
TEST(NativeType25MovingSceneCuda,CompleteCandidateCapRejectsBeforeForcePublicationOnEveryRetry) {
  Rig rig;auto limits=ObservedSource::Limits();limits.optimized_candidates=0;ASSERT_NO_THROW(rig.Initialize(limits));
  for(unsigned i=0;i<467;++i)ASSERT_NO_THROW(rig.Step());const auto before=rig.Read();
  for(unsigned retry=0;retry<2;++retry) {Attempt a;ASSERT_NO_THROW(rig.BeginMaterials(a));
    EXPECT_EQ(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly).status,n::TransactionStatus::ResourceLimit);Exact(before,rig.Read());}
}
TEST(NativeType25MovingSceneCuda,PublisherDestructionRevokesAllNativeUseBeforeDereference) {
  Rig rig;ASSERT_NO_THROW(rig.Initialize());ASSERT_NO_THROW(rig.Step());rig.publication.reset();
  EXPECT_FALSE(rig.contact.accepted().available);Attempt attempt;ASSERT_NO_THROW(rig.BeginMaterials(attempt));
  EXPECT_EQ(rig.contact.AssembleAccepted(rig.owner,attempt.token,attempt.assembly).status,n::TransactionStatus::PublicationFailure);
  rig.contact.DiscardTrial();EXPECT_EQ(rig.owner.accepted().epoch,1u);
}
TEST(NativeType25MovingSceneCuda,SourceMetadataAlignmentAndEmptySpanContractsRejectAtStartup) {
  Rig rig;ASSERT_NO_THROW(rig.Initialize(ObservedSource::Limits(),false));const auto config=ObservedSource::Config();
  for(unsigned field=0;field<6;++field) {
    auto source=rig.source.View();const auto shift=[](const auto* p){return reinterpret_cast<const unsigned char*>(p)+1;};
    if(field==0)source.selection.nodes=reinterpret_cast<const l::Node*>(shift(source.selection.nodes));
    if(field==1)source.selection.mains=reinterpret_cast<const l::Main*>(shift(source.selection.mains));
    if(field==2)source.selection.secondary=reinterpret_cast<const l::Secondary*>(shift(source.selection.secondary));
    if(field==3)source.selection.normal_to_main.offsets=reinterpret_cast<const std::uint32_t*>(shift(source.selection.normal_to_main.offsets));
    if(field==4)source.primary_parent_ids=reinterpret_cast<const std::uint64_t*>(shift(source.primary_parent_ids));
    if(field==5)source.selection.normal_to_main.entry_count=SIZE_MAX;
    n::Transaction rejected;const auto report=rejected.Initialize(config,source,rig.owner,*rig.publication,rig.fixture.physical,rig.Participants(),rig.Identity(),ObservedSource::Limits());
    EXPECT_NE(report.status,n::TransactionStatus::Ok);EXPECT_EQ(rejected.allocations().device_bytes,0u);
  }
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
} // namespace native_runtime_test
