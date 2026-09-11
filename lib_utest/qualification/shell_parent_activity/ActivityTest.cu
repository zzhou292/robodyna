// SPDX-License-Identifier: MIT
#include "../qbat_resident/ResidentFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace qbat_resident_test {
namespace {
fe::ShellFormulationParticipants Participants(Rig& rig) {
  return {rig.binding->qeph_count() ? &rig.qeph : nullptr,
          rig.binding->t3_count() ? &rig.t3 : nullptr,&rig.qbat,nullptr};
}
void CheckActivity(Rig& rig,bool expect_virgin=false) {
  const auto stamp=rig.owner.accepted();
  fe::ShellBatchDiagnostics common;
  ASSERT_EQ(rig.publication.CopyAcceptedDiagnostics(stamp,&common).status,
            fe::ShellPublicationStatus::Success);
  ASSERT_EQ(rig.publication.ValidateAcceptedActivitySources(
      rig.owner,Participants(rig),rig.binding->inventory()).status,
      fe::ShellPublicationStatus::Success);
  if (rig.binding->qeph_count()) {
    const auto count=rig.binding->qeph_count();
    std::vector<std::uint8_t> activity(count,19);
    std::vector<fe::ShellBatchFailureState> history(count);
    fe::qeph::BatchDiagnostics diagnostics,full;
    ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,activity.data(),count,&diagnostics).status,
              fe::qeph::BatchStatus::Success);
    ASSERT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,history.data(),count,&full).status,
              fe::qeph::BatchStatus::Success);
    EXPECT_EQ(diagnostics.epoch,common.qeph.epoch);
    EXPECT_EQ(diagnostics.owner_id,common.qeph.owner_id);
    for (std::size_t i=0;i<count;++i) {
      EXPECT_EQ(activity[i],history[i].active ? 1 : 0);
      if (expect_virgin) EXPECT_EQ(activity[i],1);
    }
  }
  if (rig.binding->t3_count()) {
    const auto count=rig.binding->t3_count();
    std::vector<std::uint8_t> activity(count,19);
    std::vector<fe::t3::ForceTrial> history(count);
    std::vector<fe::ShellBatchLayeredSection> sections(count);
    fe::t3::BatchDiagnostics diagnostics,full,typed;
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(stamp,activity.data(),count,&diagnostics).status,
              fe::t3::BatchStatus::Success);
    ASSERT_EQ(rig.t3.CopyAcceptedResults(stamp,history.data(),count,&full).status,
              fe::t3::BatchStatus::Success);
    ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(stamp,sections.data(),count,&typed).status,
              fe::t3::BatchStatus::Success);
    EXPECT_EQ(diagnostics.epoch,common.t3.epoch);
    EXPECT_EQ(diagnostics.owner_id,common.t3.owner_id);
    for (std::size_t i=0;i<count;++i) {
      EXPECT_EQ(activity[i],history[i].proposed_history.data().active);
      if (const auto* point=sections[i].one_point())
        EXPECT_EQ(activity[i],point->point.failure.history.point_active ? 1 : 0);
      if (expect_virgin) EXPECT_EQ(activity[i],1);
    }
  }
  std::vector<std::uint8_t> activity(rig.binding->qbat_count(),19);
  std::vector<qb::BatchResult> history(activity.size());
  qb::BatchDiagnostics diagnostics,full;
  ASSERT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,activity.data(),activity.size(),&diagnostics).status,
            qb::BatchStatus::Success);
  ASSERT_EQ(rig.qbat.CopyAcceptedResults(stamp,history.data(),history.size(),&full).status,
            qb::BatchStatus::Success);
  EXPECT_TRUE(qb::batch_detail::SameDiagnostics(diagnostics,common.qbat));
  for (std::size_t i=0;i<activity.size();++i) {
    EXPECT_EQ(activity[i],history[i].history.element_active ? 1 : 0);
    if (expect_virgin) EXPECT_EQ(activity[i],1);
  }
}
}
TEST(ShellParentActivityCuda, BoundVirginHistoryAndAcceptedCompleteFamilyIdentity) {
  Source source(true);
  qb::Batch unbound;
  const auto config=source.Config();
  ASSERT_EQ(unbound.InitializeFormulations(config,source.Scope()).status,qb::BatchStatus::Success);
  std::uint8_t flag=19;
  qb::BatchDiagnostics untouched;
  const auto before=Bytes(untouched);
  EXPECT_EQ(unbound.CopyAcceptedParentActivity(config.owner,&flag,1,&untouched).status,
            qb::BatchStatus::NotBound);
  EXPECT_EQ(flag,19);
  EXPECT_EQ(Bytes(untouched),before);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  CheckActivity(rig,true);
  auto foreign=Participants(rig);
  foreign.qbat=&unbound;
  EXPECT_EQ(rig.publication.ValidateAcceptedActivitySources(rig.owner,foreign,source.binding.inventory()).status,
            fe::ShellPublicationStatus::NotJoined);
  Source another;
  EXPECT_EQ(rig.publication.ValidateAcceptedActivitySources(
      rig.owner,Participants(rig),another.binding.inventory()).status,fe::ShellPublicationStatus::NotJoined);
  fe::FENodalState no_owner;
  EXPECT_NE(rig.publication.ValidateAcceptedActivitySources(
      no_owner,Participants(rig),source.binding.inventory()).status,fe::ShellPublicationStatus::Success);
  Prepared next;
  ASSERT_TRUE(rig.Prepare(next,1));
  CheckActivity(rig,true); // prepared state never changes the accepted flags
  ASSERT_TRUE(rig.Commit(next));
  CheckActivity(rig);
}
TEST(ShellParentActivityCuda, CompleteCapacityStaleStampAndLateReadFailureAreAtomic) {
  Source source;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  const auto stamp=rig.owner.accepted();
  std::vector<std::uint8_t> flags(source.binding.qbat_count(),19);
  qb::BatchDiagnostics diagnostics;
  const auto before=Bytes(diagnostics);
  EXPECT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,flags.data(),SIZE_MAX,&diagnostics).status,
            qb::BatchStatus::ResourceLimit);
  auto stale=stamp;
  ++stale.epoch;
  EXPECT_EQ(rig.qbat.CopyAcceptedParentActivity(stale,flags.data(),flags.size(),&diagnostics).status,
            qb::BatchStatus::StaleTrial);
  for (const auto fault : {ReadFault::LateNonfinite,ReadFault::InvalidFlag}) {
    Arm(fault,flags.size());
    EXPECT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,flags.data(),flags.size(),&diagnostics).status,
              qb::BatchStatus::NonfiniteResult);
    EXPECT_EQ(flags,std::vector<std::uint8_t>(flags.size(),19));
    EXPECT_EQ(Bytes(diagnostics),before);
  }
  CheckActivity(rig,true);
  std::uint8_t qflag=19,tflag=19;
  fe::qeph::BatchDiagnostics q;
  fe::t3::BatchDiagnostics t;
  EXPECT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,&qflag,0,&q).status,fe::qeph::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stale,&tflag,source.binding.t3_count(),&t).status,
            fe::t3::BatchStatus::StaleTrial);
  EXPECT_EQ(qflag,19);
  EXPECT_EQ(tflag,19);
}
TEST(ShellParentActivityCuda, ActualRemovalAndDiscardKeepAcceptedActivityWithHistory) {
  MidlayerSource source(true,1.e-5);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(),false,false,0x1p-12));
  CheckActivity(rig,true);
  bool removed=false,t3_removed=false;
  for (unsigned step=0;step<24;++step) {
    Prepared candidate;
    ASSERT_TRUE(rig.Prepare(candidate,30));
    CheckActivity(rig);
    if (step==0) {
      rig.Discard();
      CheckActivity(rig,true);
      ASSERT_TRUE(rig.Prepare(candidate,30));
    }
    ASSERT_TRUE(rig.Commit(candidate));
    CheckActivity(rig);
    for (const auto& row:candidate.qbat) removed|=!row.history.element_active;
    std::uint8_t active=19;
    fe::t3::BatchDiagnostics diagnostics;
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&active,1,&diagnostics).status,
              fe::t3::BatchStatus::Success);
    t3_removed|=active==0;
  }
  EXPECT_TRUE(removed);
  EXPECT_TRUE(t3_removed);
}
} // namespace qbat_resident_test
