#include "FailureResidentFixture.h"

namespace resident_failure_test {
bool Read(Rig& rig,Frame& output,const fe::ShellBatchDiagnostics* candidate) {
  Frame next;
  if(!mixed::ReadFrame(rig,next.material,candidate))return false;
  qe::BatchDiagnostics qd;
  tr::BatchDiagnostics td;
  const auto qr=candidate?rig.qeph.CopyPreparedFailureHistory(candidate->qeph,next.qfailure.data(),Parents):
    rig.qeph.CopyAcceptedFailureHistory(rig.owner.accepted(),next.qfailure.data(),Parents,&qd);
  const auto tt=candidate?rig.t3.CopyPreparedFailureHistory(candidate->t3,next.tfailure.data(),Parents):
    rig.t3.CopyAcceptedFailureHistory(rig.owner.accepted(),next.tfailure.data(),Parents,&td);
  EXPECT_EQ(qr.status,qe::BatchStatus::Success)<<qr.message;
  EXPECT_EQ(tt.status,tr::BatchStatus::Success)<<tt.message;
  if(qr.status!=qe::BatchStatus::Success||tt.status!=tr::BatchStatus::Success)return false;
  if(!candidate) {
    EXPECT_EQ(qd.epoch,rig.owner.accepted().epoch);
    EXPECT_EQ(td.epoch,qd.epoch);
    EXPECT_EQ(qd.phase,qe::BatchPhase::Accepted);
    EXPECT_EQ(td.phase,tr::BatchPhase::Accepted);
  }
  output=next;
  return true;
}
bool Evaluate(Rig& rig,const Prepared& prepared,Frame& output) {
  mixed::Frame material;
  if(!mixed::Evaluate(rig,prepared,material))return false;
  return Read(rig,output,&material.diagnostics);
}
namespace {
void Sidecar(const fe::ShellBatchFailureState& a,const fe::ShellBatchFailureState& b) {
  EXPECT_EQ(a.policy,b.policy);
  EXPECT_EQ(a.active,b.active);
  std::vector<double> x,y;
  for(unsigned p=0;p<3;++p) {
    EXPECT_EQ(a.point[p].point_active,b.point[p].point_active);
    failure_force_test::Append(x,a.point[p].damage);
    failure_force_test::Append(y,b.point[p].damage);
    failure_force_test::Append(x,a.point[p].failure_time_s);
    failure_force_test::Append(y,b.point[p].failure_time_s);
    failure_force_test::Append(x,a.current_force_point[p].stress);
    failure_force_test::Append(y,b.current_force_point[p].stress);
  }
  failure_force_test::Exact(x,y);
}
void Section(const fe::ShellBatchLayeredSection& a,const fe::ShellBatchLayeredSection& b) {
  EXPECT_EQ(a.law(),b.law());
  std::vector<double> x,y;
  if(a.elastic()) {
    ASSERT_NE(b.elastic(),nullptr);
    for(unsigned p=0;p<3;++p) {
      failure_force_test::Append(x,a.elastic()->point[p].stress);
      failure_force_test::Append(y,b.elastic()->point[p].stress);
    }
  } else {
    ASSERT_NE(a.plastic(),nullptr);
    ASSERT_NE(b.plastic(),nullptr);
    failure_force_test::Append(x,a.plastic()->history);
    failure_force_test::Append(y,b.plastic()->history);
    failure_force_test::Append(x,a.plastic()->diagnostics);
    failure_force_test::Append(y,b.plastic()->diagnostics);
    failure_force_test::Append(x,a.plastic()->cumulative_plastic_work_J);
    failure_force_test::Append(y,b.plastic()->cumulative_plastic_work_J);
  }
  failure_force_test::Exact(x,y);
}
}
void Same(const Frame& a,const Frame& b) {
  for(unsigned e=0;e<Parents;++e) {
    failure_force_test::Exact(failure_force_test::ForceValues(a.material.qforce[e]),failure_force_test::ForceValues(b.material.qforce[e]));
    failure_force_test::Exact(failure_force_test::ForceValues(a.material.tforce[e]),failure_force_test::ForceValues(b.material.tforce[e]));
    failure_force_test::Exact(failure_force_test::Diagnostics(a.material.qforce[e].diagnostics),failure_force_test::Diagnostics(b.material.qforce[e].diagnostics));
    failure_force_test::Exact(failure_force_test::Diagnostics(a.material.tforce[e].diagnostics),failure_force_test::Diagnostics(b.material.tforce[e].diagnostics));
    Section(a.material.qsection[e],b.material.qsection[e]);
    Section(a.material.tsection[e],b.material.tsection[e]);
    Sidecar(a.qfailure[e],b.qfailure[e]);
    Sidecar(a.tfailure[e],b.tfailure[e]);
  }
}
} // namespace resident_failure_test
