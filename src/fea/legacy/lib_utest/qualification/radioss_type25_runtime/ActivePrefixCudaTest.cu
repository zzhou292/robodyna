// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FullLedgerRig.h"
#include "../qt_mapped/TypedValues.h"
#include "../qbat_resident/ResultValues.h"
#include "lib_src/elements/publication/PhysicalState.h"
#include <cstring>
#include "../radioss_type25_local_geometry/Assertions.h"
namespace type25_source_test {
namespace {
// Reuse typed history value helpers; never compare object padding.
std::vector<std::uint64_t> History(FullLedgerRig& rig) {
  std::vector<std::uint64_t> values;
  const auto stamp=rig.owner.accepted();const auto count=rig.m.size();
  std::vector<double> nodes(19*count),coefficients(2*count+3);fe::NodalStamp copied;
  Check(rig.owner.CopyAccepted({nodes.data(),nodes.data()+3*count,count,nodes.data()+6*count,
      nodes.data()+10*count,nodes.data()+13*count,nodes.data()+16*count},&copied));
  Check(rig.owner.CopyAcceptedCin({coefficients.data(),coefficients.data()+count,
      coefficients.data()+2*count,coefficients.data()+2*count+1,coefficients.data()+2*count+2,count,1},&copied));
  for(const auto& range:{nodes,coefficients})for(double x:range) {
    std::uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));values.push_back(bits);
  }
  std::vector<fe::ShellBatchLayeredSection> q(rig.fixture.shells.qeph_count()),s(rig.fixture.shells.t3_count());
  fe::qeph::BatchDiagnostics qd;fe::t3::BatchDiagnostics td;fe::qbat::BatchDiagnostics bd;
  Check(rig.qeph.CopyAcceptedLayeredSectionHistory(stamp,q.data(),q.size(),&qd));
  Check(rig.t3.CopyAcceptedLayeredSectionHistory(stamp,s.data(),s.size(),&td));
  for(const auto& row:q)qt_mapped_test::Add(values,row);
  for(const auto& row:s)qt_mapped_test::Add(values,row);
  fe::qbat::BatchResult qb;Check(rig.qbat.CopyAcceptedResults(stamp,&qb,1,&bd));
  const auto b=qbat_resident_test::ResultValues(qb);values.insert(values.end(),b.begin(),b.end());
  return values;
}
void Warm(FullLedgerRig& rig) {
  for(unsigned i=0;i<2;++i) {
    FullLedgerAttempt a;rig.Begin(a);Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    rig.Prepare(a);rig.Seal(a);Check(rig.Commit(a));
  }
}
}
TEST(NativeActivePrefixCuda, RealQbatOnePointAndAllContributorsPublishWithoutDisablingFailure) {
  FullLedgerRig rig(true);
  ASSERT_NO_THROW(rig.Initialize());
  ASSERT_EQ(rig.fixture.shells.qbat_count(),1u);
  ASSERT_NO_THROW(Warm(rig));
  FullLedgerAttempt a;
  ASSERT_NO_THROW(rig.Begin(a));
  ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
  EXPECT_GT(rig.contact.last_diagnostics().active_forces,0u);
  ASSERT_NO_THROW(rig.Prepare(a));
  ASSERT_NO_THROW(rig.Seal(a));
  ASSERT_NO_THROW(Check(rig.Commit(a)));
  EXPECT_EQ(rig.owner.accepted().epoch,3u);
  EXPECT_EQ(rig.contact.accepted().generation,3u);
  EXPECT_EQ(a.common.qbat.active_count,1u);
}
TEST(NativeActivePrefixCuda, FailureAfterActiveContactDiscardsWholeAttemptAndExactlyRetries) {
  FullLedgerRig rig(true,1e-3);
  ASSERT_NO_THROW(rig.Initialize());
  ASSERT_NO_THROW(Warm(rig));
  const auto stamp=rig.owner.accepted();const auto old=rig.contact.accepted();
  std::vector<std::uint64_t> before;
  ASSERT_NO_THROW(before=History(rig));
  fe::ShellPhysicalDiagnostics diagnostics;
  ASSERT_NO_THROW(Check(rig.publication.CopyAcceptedPhysicalDiagnostics(stamp,&diagnostics)));
  const auto rows=rig.fixture.secondary.size();
  std::vector<n::NativeGeometryHistory> original_history(rows);std::vector<int> original_flags(rows);
  fe::NativeContactPublicationSnapshot history_stamp;
  ASSERT_NO_THROW(Check(rig.contact.CopyAccepted({original_history.data(),original_flags.data(),rows},&history_stamp)));
  std::vector<double> force;
  for(unsigned retry=0;retry<2;++retry) {
    FullLedgerAttempt a;
  ASSERT_NO_THROW(rig.Begin(a));
    const auto node=rig.fixture.domain.Find(14);
    const double load=2*rig.m[node]*.001/(FullLedgerRig::Dt*FullLedgerRig::Dt);
    ASSERT_NO_THROW(Check(cudaMemcpyAsync(a.assembly.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,a.assembly.stream)));
    ASSERT_NO_THROW(Check(cudaStreamSynchronize(a.assembly.stream)));
    ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
    ASSERT_GT(rig.contact.last_diagnostics().active_forces,0u);
    std::vector<double> current;
  ASSERT_NO_THROW(current=rig.Force(a));
    if(!retry)force=current;
    else {ASSERT_EQ(force.size(),current.size());EXPECT_EQ(std::memcmp(force.data(),current.data(),force.size()*sizeof(double)),0);}
    ASSERT_NO_THROW(rig.Prepare(a));
    const auto rejected=rig.contact.SealCandidate(rig.owner,a.token,a.prepared,a.common,&a.contact);
    EXPECT_EQ(rejected.status,n::TransactionStatus::ActivityChange)<<rejected.message;
    EXPECT_EQ(rejected.row,0u);
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
    EXPECT_EQ(rig.contact.accepted().generation,old.generation);
    EXPECT_EQ(rig.contact.accepted().selectors.history,old.selectors.history);
    std::vector<n::NativeGeometryHistory> history(rows);std::vector<int> flags(rows);
    fe::NativeContactPublicationSnapshot repeated_stamp;
    ASSERT_NO_THROW(Check(rig.contact.CopyAccepted({history.data(),flags.data(),rows},&repeated_stamp)));
    EXPECT_EQ(flags,original_flags);
    for(std::size_t row=0;row<rows;++row)type25_geometry_test::Same(history[row],original_history[row]);
    std::vector<std::uint64_t> after;
  ASSERT_NO_THROW(after=History(rig));EXPECT_EQ(after,before);
    fe::ShellPhysicalDiagnostics remaining;
    ASSERT_NO_THROW(Check(rig.publication.CopyAcceptedPhysicalDiagnostics(stamp,&remaining)));
    EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(diagnostics,remaining));
    EXPECT_NE(rig.Commit(a).status,fe::ShellPublicationStatus::Success);
    rig.Discard();
  }
}
} // namespace type25_source_test
