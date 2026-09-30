#include "MixedResidentFixture.h"

namespace mixed_layered_test {
bool ReadFrame(Rig& r,Frame& output,const fe::ShellBatchDiagnostics* candidate) {
  Frame next;q::BatchReport qr;t::BatchReport tr;
  if(candidate) {
    next.diagnostics=*candidate;
    qr=r.qeph.CopyPreparedResults(candidate->qeph,next.qforce.data(),Parents);
    tr=r.t3.CopyPreparedResults(candidate->t3,next.tforce.data(),Parents);
  } else {
    qr=r.qeph.CopyAcceptedResults(r.owner.accepted(),next.qforce.data(),Parents,&next.diagnostics.qeph);
    tr=r.t3.CopyAcceptedResults(r.owner.accepted(),next.tforce.data(),Parents,&next.diagnostics.t3);
  }
  EXPECT_EQ(qr.status,q::BatchStatus::Success);EXPECT_EQ(tr.status,t::BatchStatus::Success);
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success)return false;
  q::BatchDiagnostics qd;t::BatchDiagnostics td;
  qr=candidate?r.qeph.CopyPreparedLayeredSectionHistory(candidate->qeph,next.qsection.data(),Parents):
    r.qeph.CopyAcceptedLayeredSectionHistory(r.owner.accepted(),next.qsection.data(),Parents,&qd);
  tr=candidate?r.t3.CopyPreparedLayeredSectionHistory(candidate->t3,next.tsection.data(),Parents):
    r.t3.CopyAcceptedLayeredSectionHistory(r.owner.accepted(),next.tsection.data(),Parents,&td);
  EXPECT_EQ(qr.status,q::BatchStatus::Success);EXPECT_EQ(tr.status,t::BatchStatus::Success);
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success)return false;
  if(!candidate) { EXPECT_EQ(qd.epoch,r.owner.accepted().epoch);EXPECT_EQ(td.epoch,qd.epoch); }
  output=next;return true;
}
bool Evaluate(Rig& r,const Prepared& p,Frame& output) {
  fe::ShellBatchDiagnostics candidate;
  const auto qr=r.qeph.EvaluateCandidate(p.view,&candidate.qeph);
  const auto tr=r.t3.EvaluateCandidate(p.view,&candidate.t3);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success)return false;
  fe::ShellBatchDiagnostics joined;
  const auto report=r.publication.Prepare(r.owner,p.token,candidate.qeph,candidate.t3,&joined);
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  return report.status==fe::ShellPublicationStatus::Success&&ReadFrame(r,output,&joined);
}
bool Commit(Rig& r,const Prepared& p,const Frame& frame) {
  const auto& d=frame.diagnostics.qeph;
  const auto report=r.publication.Commit(r.owner,p.token,frame.diagnostics,
    {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  return report.status==fe::ShellPublicationStatus::Success;
}
namespace {
void SameSection(const fe::ShellBatchLayeredSection& a,const fe::ShellBatchLayeredSection& b) {
  ASSERT_EQ(a.law(),b.law());
  if(a.elastic()) { ASSERT_NE(b.elastic(),nullptr);EXPECT_EQ(Bytes(*a.elastic()),Bytes(*b.elastic())); }
  else { ASSERT_NE(a.plastic(),nullptr);ASSERT_NE(b.plastic(),nullptr);EXPECT_EQ(Bytes(*a.plastic()),Bytes(*b.plastic())); }
}
}
void SameFrame(const Frame& a,const Frame& b) {
  for(unsigned i=0;i<Parents;++i) {
    EXPECT_EQ(Bytes(a.qforce[i]),Bytes(b.qforce[i]));to::Exact(a.tforce[i],b.tforce[i]);
    SameSection(a.qsection[i],b.qsection[i]);SameSection(a.tsection[i],b.tsection[i]);
  }
}
} // namespace mixed_layered_test
