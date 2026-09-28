// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellRemovalFixture.h"
#include "RuntimeObservation.h"
#include "../radioss_type25_local_geometry/Assertions.h"
#include <cstring>
namespace type25_source_test {
namespace {
struct AcceptedHistory {
  std::vector<n::NativeGeometryHistory> rows;
  std::vector<int> flags;
  fe::NativeContactPublicationSnapshot snapshot;
  explicit AcceptedHistory(ShellRemovalRig& rig):rows(rig.fixture.secondary.size()),flags(rows.size()) {
    Check(rig.contact.CopyAccepted({rows.data(),flags.data(),rows.size()},&snapshot));
  }
};
void Same(const AcceptedHistory& a,const AcceptedHistory& b) {
  ASSERT_EQ(a.rows.size(),b.rows.size());EXPECT_EQ(a.flags,b.flags);
  for(std::size_t row=0;row<a.rows.size();++row)type25_geometry_test::Same(a.rows[row],b.rows[row]);
  EXPECT_EQ(a.snapshot.generation,b.snapshot.generation);
  EXPECT_EQ(a.snapshot.selectors.activity,b.snapshot.selectors.activity);
  EXPECT_EQ(a.snapshot.selectors.activity_generation,b.snapshot.selectors.activity_generation);
}
}
TEST(NativeShellRemovalCuda, RealRemovalCommitsThenRebuildsSearchAndContinues) {
  ShellRemovalRig rig;ASSERT_NO_THROW(rig.InitializeRemoval());ASSERT_NO_THROW(rig.Warm());
  const auto before=rig.contact.accepted();
  FullLedgerAttempt a;ASSERT_NO_THROW(rig.Begin(a));ASSERT_NO_THROW(rig.RemovalLoad(a));
  ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
  ASSERT_GT(rig.contact.last_diagnostics().active_forces,0u);
  ASSERT_NO_THROW(rig.Prepare(a));ASSERT_NO_THROW(rig.Seal(a));
  EXPECT_TRUE(rig.contact.last_diagnostics().activity_changed);
  EXPECT_EQ(rig.contact.last_diagnostics().activity_removed_mains,2u);
  // Candidate staging cannot change the accepted source or time.
  EXPECT_EQ(rig.contact.accepted().selectors.activity,before.selectors.activity);
  EXPECT_EQ(rig.owner.accepted().epoch,2u);
  ASSERT_NO_THROW(Check(rig.Commit(a)));
  const auto changed=rig.contact.accepted();
  EXPECT_EQ(changed.selectors.activity,before.selectors.activity^1u);
  EXPECT_EQ(changed.selectors.activity_generation,before.selectors.activity_generation+1);
  EXPECT_EQ(changed.selectors.reference_activity_generation,before.selectors.activity_generation);
  for(unsigned step=0;step<3;++step) {
    FullLedgerAttempt next;ASSERT_NO_THROW(rig.Begin(next));
    ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,next.token,next.assembly)));
    if(!step)EXPECT_TRUE(rig.contact.last_diagnostics().reference_rebuilt);
    ASSERT_NO_THROW(rig.Prepare(next));ASSERT_NO_THROW(rig.Seal(next));
    EXPECT_FALSE(rig.contact.last_diagnostics().activity_changed);
    ASSERT_NO_THROW(Check(rig.Commit(next)));
    EXPECT_EQ(rig.contact.accepted().selectors.activity_generation,changed.selectors.activity_generation);
    EXPECT_EQ(rig.contact.accepted().selectors.reference_activity_generation,changed.selectors.activity_generation);
  }
  EXPECT_EQ(rig.owner.accepted().epoch,6u);
}
TEST(NativeShellRemovalCuda, LateCommonRejectionPreservesHistoryAndRetriesRemovalExactly) {
  ShellRemovalRig rig;ASSERT_NO_THROW(rig.InitializeRemoval());ASSERT_NO_THROW(rig.Warm());
  AcceptedHistory before(rig);const auto stamp=rig.owner.accepted();std::vector<double> force;
  n::TransactionDiagnostics first;
  for(unsigned retry=0;retry<2;++retry) {
    FullLedgerAttempt a;ASSERT_NO_THROW(rig.Begin(a));ASSERT_NO_THROW(rig.RemovalLoad(a));
    ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
    std::vector<double> actual;ASSERT_NO_THROW(actual=rig.Force(a));
    if(!retry)force=actual;
    else {ASSERT_EQ(actual.size(),force.size());EXPECT_EQ(std::memcmp(actual.data(),force.data(),force.size()*sizeof(double)),0);}
    ASSERT_NO_THROW(rig.Prepare(a));ASSERT_NO_THROW(rig.Seal(a));
    const auto result=rig.contact.last_diagnostics();ASSERT_TRUE(result.activity_changed);
    if(!retry) {
      first=result;EXPECT_NE(rig.Commit(a,false).status,fe::ShellPublicationStatus::Success);
      rig.Discard();AcceptedHistory after(rig);Same(before,after);
      EXPECT_TRUE(fe::trial_identity::SameStamp(rig.owner.accepted(),stamp));
    } else {
      EXPECT_EQ(result.activity_affected_events,first.activity_affected_events);
      EXPECT_EQ(result.activity_removed_events,first.activity_removed_events);
      EXPECT_EQ(result.activity_removed_mains,first.activity_removed_mains);
      EXPECT_EQ(result.activity_orphan_secondaries,first.activity_orphan_secondaries);
      ASSERT_NO_THROW(Check(rig.Commit(a)));
    }
  }
  ASSERT_NO_THROW(rig.Warm(1));
}
TEST(NativeShellRemovalCuda, NoRemovalPreservesLegacyForceAndHistoryValues) {
  FullLedgerRig old(true);ShellRemovalRig next(2.5);
  ASSERT_NO_THROW(old.Initialize());ASSERT_NO_THROW(next.InitializeRemoval());
  for(unsigned step=0;step<4;++step) {
    FullLedgerAttempt a,b;ASSERT_NO_THROW(old.Begin(a));ASSERT_NO_THROW(next.Begin(b));
    ASSERT_NO_THROW(Check(old.contact.AssembleAccepted(old.owner,a.token,a.assembly)));
    ASSERT_NO_THROW(Check(next.contact.AssembleAccepted(next.owner,b.token,b.assembly)));
    std::vector<double> x,y;ASSERT_NO_THROW(x=old.Force(a));ASSERT_NO_THROW(y=next.Force(b));
    ASSERT_EQ(x.size(),y.size());EXPECT_EQ(std::memcmp(x.data(),y.data(),x.size()*sizeof(double)),0);
    ASSERT_NO_THROW(old.Prepare(a));ASSERT_NO_THROW(next.Prepare(b));
    ASSERT_NO_THROW(old.Seal(a));ASSERT_NO_THROW(next.Seal(b));
    ASSERT_NO_THROW(Check(old.Commit(a)));ASSERT_NO_THROW(Check(next.Commit(b)));
    EXPECT_FALSE(next.contact.last_diagnostics().activity_changed);
    EXPECT_EQ(next.contact.accepted().selectors.activity_generation,1u);
    const auto count=old.fixture.secondary.size();
    std::vector<n::NativeGeometryHistory> h1(count),h2(count);std::vector<int> f1(count),f2(count);
    fe::NativeContactPublicationSnapshot s1,s2;
    ASSERT_NO_THROW(Check(old.contact.CopyAccepted({h1.data(),f1.data(),count},&s1)));
    ASSERT_NO_THROW(Check(next.contact.CopyAccepted({h2.data(),f2.data(),count},&s2)));
    EXPECT_EQ(f1,f2);for(std::size_t i=0;i<count;++i)type25_geometry_test::Same(h1[i],h2[i]);
  }
}
}
