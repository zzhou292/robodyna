#include "Fixture.h"
namespace global_law1_test {
TEST(GlobalLaw1Native,QephNptZeroAndThreeRemainDistinctBendingBranches) {
  auto input=Quad();nq::Reference r;ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),r),nq::Status::kSuccess);
  nq::History h;ASSERT_EQ(nq::InitializeHistory(r,{},h),nq::Status::kSuccess);
  q::ReferenceData pr;
  ASSERT_EQ(q::InitializeReference(input,pr),q::Status::kSuccess);
  const auto ph=QHistory(pr);auto in=qeph_force_port_test::Next(input,ph);
  qeph_force_port_test::ApplyMode(in,5,100.);nq::ForceTrial global_result;native::QephTrial layered;
  ASSERT_EQ(global::Evaluate(r,h,qeph_kinematics_test::NativeInterval(in),1,global_result),nq::Status::kSuccess);
  ASSERT_EQ(native::Evaluate(r,{h,{}},qeph_kinematics_test::NativeInterval(in),layered),nq::Status::kSuccess);
  const double delta=std::abs(global_result.proposed_history.data().bending_stress[0]-layered.shell.proposed_history.data().bending_stress[0]);
  EXPECT_GT(delta,0.);path::RecordNumber("npt0_npt3_bending_stress_delta_pa",delta);
  path::RecordNumber("npt0_npt3_max_force_delta_n",ForceDifference(global_result,layered.shell));
}
TEST(GlobalLaw1Native,T3NptZeroAndThreeRemainDistinctBendingBranches) {
  auto input=Triangle();nt::Reference r;ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),r),nt::Status::kSuccess);
  nt::History h;ASSERT_EQ(nt::InitializeHistory(r,{},h),nt::Status::kSuccess);
  auto in=t3_port_test::Interval(input,1e-6);in.sample_index=1;t3_force_port_test::Mode(in,5,100.);
  nt::ForceTrial global_result;native::T3Trial layered;
  ASSERT_EQ(global::Evaluate(r,h,t3_port_test::Native(in),1,global_result),nt::Status::kSuccess);
  ASSERT_EQ(native::Evaluate(r,{h,{}},t3_port_test::Native(in),layered),nt::Status::kSuccess);
  const double delta=std::abs(global_result.proposed_history.data().bending_stress[0]-layered.shell.proposed_history.data().bending_stress[0]);
  EXPECT_GT(delta,0.);path::RecordNumber("npt0_npt3_bending_stress_delta_pa",delta);
  path::RecordNumber("npt0_npt3_max_force_delta_n",ForceDifference(global_result,layered.shell));
}
} // namespace global_law1_test
