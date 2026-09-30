#include "LayeredNativePath.h"
#include <cstring>
#include <limits>

namespace layered_j2_test::recurrence {
namespace {
template<class T> auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes{};std::memcpy(bytes.data(),&value,sizeof value);return bytes;
}
template<class Trial> void Exact(const Trial& a,const Trial& b) {
  EXPECT_EQ(a.points,b.points);
  const auto& x=a.shell.proposed_history.data();const auto& y=b.shell.proposed_history.data();
  EXPECT_EQ(x.stress,y.stress);EXPECT_EQ(x.material_stress,y.material_stress);
  EXPECT_EQ(x.bending_stress,y.bending_stress);EXPECT_EQ(x.strain_curvature,y.strain_curvature);
  EXPECT_EQ(x.thickness,y.thickness);EXPECT_EQ(x.internal_work,y.internal_work);
  for(unsigned n=0;n<a.shell.internal_force.size();++n) {
    for(auto pair:{std::make_pair(a.shell.internal_force[n],b.shell.internal_force[n]),
                   std::make_pair(a.shell.internal_couple[n],b.shell.internal_couple[n])}) {
      EXPECT_EQ(pair.first.x,pair.second.x);EXPECT_EQ(pair.first.y,pair.second.y);EXPECT_EQ(pair.first.z,pair.second.z);
    }
  }
  EXPECT_EQ(a.section.plastic_work_increment_j,b.section.plastic_work_increment_j);
  EXPECT_EQ(a.shell.proposed_history.stamp().time,b.shell.proposed_history.stamp().time);
  EXPECT_EQ(a.shell.proposed_history.stamp().sample_index,b.shell.proposed_history.stamp().sample_index);
}
template<class Ref,class History,class Interval,class Trial,class Status>
void RejectionControls(const Ref& r,const History& base,const Interval& good,
                       const Trial& expected,Status success) {
  Trial out=expected;const auto before=Bytes(out),accepted=Bytes(base);auto interval=good;
  interval.sample_index--;
  EXPECT_NE(oracle::Evaluate(r,base,interval,Law(),out),success);EXPECT_EQ(Bytes(out),before);
  interval=good;interval.base_time+=interval.dt;
  EXPECT_NE(oracle::Evaluate(r,base,interval,Law(),out),success);EXPECT_EQ(Bytes(out),before);
  auto law=Law();std::array<double,46> invalid_curve{};
  std::copy_n(law.yield_stress,law.count,invalid_curve.begin());
  invalid_curve.back()=std::numeric_limits<double>::quiet_NaN();law.yield_stress=invalid_curve.data();
  EXPECT_NE(oracle::Evaluate(r,base,good,law,out),success);EXPECT_EQ(Bytes(out),before);
  // The first two native layers complete; finite third-layer input overflows
  // actual SIGEPS44C arithmetic. Private native mutation must not be published.
  auto late=base;late.points[2][0]=std::numeric_limits<double>::max()/16;
  EXPECT_NE(oracle::Evaluate(r,late,good,Law(),out),success);EXPECT_EQ(Bytes(out),before);
  EXPECT_EQ(Bytes(base),accepted);
  EXPECT_EQ(oracle::Evaluate(r,base,good,Law(),out),success);Exact(out,expected);
  // Borrowing a curve from destination storage is prohibited even on success.
  law=Law();law.yield_stress=out.points[0].data();
  const auto alias_before=Bytes(out);
  EXPECT_NE(oracle::Evaluate(r,base,good,law,out),success);EXPECT_EQ(Bytes(out),alias_before);
}
template<class Trial> double StressDistance(const Trial& a,const Trial& b) {
  double maximum=0;for(unsigned p=0;p<3;++p) for(unsigned c=0;c<5;++c)
    maximum=std::max(maximum,std::abs(a.points[p][c]-b.points[p][c]));return maximum;
}
}
TEST(ShellLayeredNativeControls,QephRejectsLateMutationAndDetectsWrongPhaseOrResetHistory) {
  const auto material=cv::source::Prepare();auto input=qeph_startup_test::Case(5);
  cv::SourceMaterial(input,material);nq::Reference r;oracle::QephHistory base;
  ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),r),nq::Status::kSuccess);
  ASSERT_EQ(nq::InitializeHistory(r,{},base.shell),nq::Status::kSuccess);
  Path path{true};
  for(;path.step<112;++path.step) {
    oracle::QephTrial next;ASSERT_EQ(oracle::Evaluate(r,base,
      qeph_kinematics_test::NativeInterval(path.Interval(input)),Law(),next),nq::Status::kSuccess);
    base={next.shell.proposed_history,next.points};
  }
  ASSERT_GT(std::max(base.points[0][5],base.points[2][5]),1e-3);
  const auto in=qeph_kinematics_test::NativeInterval(path.Interval(input));
  oracle::QephTrial correct,wrong,reset;
  ASSERT_EQ(oracle::Evaluate(r,base,in,Law(),correct),nq::Status::kSuccess);
  ASSERT_EQ(oracle::Evaluate(r,base,qeph_kinematics_test::NativeInterval(path.Interval(input,true)),Law(),wrong),nq::Status::kSuccess);
  EXPECT_GT(StressDistance(correct,wrong),100.);
  oracle::QephHistory erased;nq::HistoryValues zero;zero.thickness=base.shell.data().thickness;
  ASSERT_EQ(nq::PreparePrescribedHistory(r,zero,base.shell.stamp(),erased.shell),nq::Status::kSuccess);
  ASSERT_EQ(oracle::Evaluate(r,erased,in,Law(),reset),nq::Status::kSuccess);
  EXPECT_GT(StressDistance(correct,reset),1e7);EXPECT_NE(correct.points[0][6],reset.points[0][6]);
  RejectionControls(r,base,in,correct,nq::Status::kSuccess);
  oracle::QephTrial retry;ASSERT_EQ(oracle::Evaluate(r,base,in,Law(),retry),nq::Status::kSuccess);
  EXPECT_EQ(retry.shell.proposed_history.data().stabilization,correct.shell.proposed_history.data().stabilization);
  EXPECT_EQ(retry.shell.proposed_history.data().hourglass_viscous_work,correct.shell.proposed_history.data().hourglass_viscous_work);
}
TEST(ShellLayeredNativeControls,T3RejectsLateMutationAndDetectsWrongPhaseOrResetHistory) {
  const auto material=cv::source::Prepare();auto input=t3_port_test::Triangle(.02,1);
  cv::SourceMaterial(input,material);nt::Reference r;oracle::T3History base;
  ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),r),nt::Status::kSuccess);
  ASSERT_EQ(nt::InitializeHistory(r,{},base.shell),nt::Status::kSuccess);
  Path path{true};
  for(;path.step<112;++path.step) {
    oracle::T3Trial next;ASSERT_EQ(oracle::Evaluate(r,base,
      t3_port_test::Native(path.Interval(input)),Law(),next),nt::Status::kSuccess);
    base={next.shell.proposed_history,next.points};
  }
  ASSERT_GT(std::max(base.points[0][5],base.points[2][5]),1e-3);
  const auto in=t3_port_test::Native(path.Interval(input));oracle::T3Trial correct,wrong,reset;
  ASSERT_EQ(oracle::Evaluate(r,base,in,Law(),correct),nt::Status::kSuccess);
  ASSERT_EQ(oracle::Evaluate(r,base,t3_port_test::Native(path.Interval(input,true)),Law(),wrong),nt::Status::kSuccess);
  EXPECT_GT(StressDistance(correct,wrong),100.);
  oracle::T3History erased;nt::HistoryValues zero;zero.thickness=base.shell.data().thickness;
  ASSERT_EQ(nt::PreparePrescribedHistory(r,zero,base.shell.stamp(),erased.shell),nt::Status::kSuccess);
  ASSERT_EQ(oracle::Evaluate(r,erased,in,Law(),reset),nt::Status::kSuccess);
  EXPECT_GT(StressDistance(correct,reset),1e7);EXPECT_NE(correct.points[0][6],reset.points[0][6]);
  RejectionControls(r,base,in,correct,nt::Status::kSuccess);
}
} // namespace layered_j2_test::recurrence
