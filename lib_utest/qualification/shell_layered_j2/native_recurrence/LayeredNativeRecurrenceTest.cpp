#include "LayeredNativePath.h"

namespace layered_j2_test::recurrence {
namespace {
struct Scenario {bool rotate;unsigned refinement;bool rate;};
constexpr Scenario Scenarios[]{{false,1,false},{false,1,true},{false,2,true},{true,1,true},{true,2,true}};
void Yielded(const sec::ShellLayeredJ2History& h,bool rate) {
  if(rate) {cv::Yielded(h);return;}
  EXPECT_GT(std::max(h.point[0].plastic_strain,h.point[2].plastic_strain),1e-3);
  EXPECT_GT(std::abs(h.point[0].plastic_strain-h.point[2].plastic_strain),1e-5);
  for(const auto& p:h.point) EXPECT_EQ(p.filtered_rate_per_s,0.);
}
}

TEST(ShellLayeredNativeRecurrence,QephIndependentYieldedRotationAndUnloadingHistories) {
  for(const auto scenario:Scenarios) {
    const auto [rotate,refinement,rate]=scenario;
    const auto material=cv::source::Prepare(rate);
    SCOPED_TRACE(rotate);
    SCOPED_TRACE(refinement);
    SCOPED_TRACE(rate);
    auto input=qeph_startup_test::Case(5);cv::SourceMaterial(input,material);
    q::ReferenceData r;nq::Reference nr;
    ASSERT_EQ(q::InitializeReference(input,r),q::Status::kSuccess);
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),nr),nq::Status::kSuccess);
    q::LayeredJ2History accepted;oracle::QephHistory native;
    ASSERT_EQ(q::InitializeLayeredJ2History(r,material,{},accepted),q::Status::kSuccess);
    ASSERT_EQ(nq::InitializeHistory(nr,{},native.shell),nq::Status::kSuccess);
    Path path{rotate,refinement};double plastic_work=0,preload_pla=0,preload_stress=0;
    for(;path.step<160*refinement;++path.step) {
      SCOPED_TRACE(path.step);
      const auto in=path.Interval(input);q::LayeredJ2ForceTrial actual;oracle::QephTrial expected;
      ASSERT_EQ(q::EvaluateLayeredJ2Force(r,material,accepted,in,actual),q::Status::kSuccess);
      ASSERT_EQ(oracle::Evaluate(nr,native,qeph_kinematics_test::NativeInterval(in),Law(rate),expected),nq::Status::kSuccess);
      qeph_force_port_test::ForceAgreement(actual.force,expected.shell,input,in,cv::Tolerance);
      Sections(actual.proposed_section,expected.points);
      Diagnostics(actual.section_diagnostics,expected.section,expected.shell.kinematics.area,
                  expected.shell.diagnostics.effective_thickness);
      ASSERT_FALSE(::testing::Test::HasFailure());
      plastic_work+=expected.section.plastic_work_increment_j;
      accepted={actual.force.proposed_history,actual.proposed_section};
      native={expected.shell.proposed_history,expected.points};
      if(path.step+1==64*refinement) {
        Yielded(accepted.section,rate);preload_pla=expected.section.max_plastic_strain;
        preload_stress=native.points[0][0];
        EXPECT_NE(native.shell.data().thickness,input.thickness);
      }
    }
    EXPECT_GT(plastic_work,1e-4);EXPECT_GE(native.points[0][5]+native.points[2][5],preload_pla);
    EXPECT_GT(std::abs(native.points[0][0]-preload_stress),1e7);
    double hg=0;for(double h:native.shell.data().stabilization) hg+=std::abs(h);
    EXPECT_GT(hg,1e4);EXPECT_GT(std::abs(native.shell.data().hourglass_viscous_work),1e-8);
  }
}
TEST(ShellLayeredNativeRecurrence,T3IndependentYieldedRotationAndUnloadingHistories) {
  for(const auto scenario:Scenarios) {
    const auto [rotate,refinement,rate]=scenario;
    const auto material=cv::source::Prepare(rate);
    SCOPED_TRACE(rotate);
    SCOPED_TRACE(refinement);
    SCOPED_TRACE(rate);
    auto input=t3_port_test::Triangle(.02,1);cv::SourceMaterial(input,material);
    t::ReferenceData r;nt::Reference nr;
    ASSERT_EQ(t::InitializeReference(input,r),t::Status::kSuccess);
    ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),nr),nt::Status::kSuccess);
    t::LayeredJ2History accepted;oracle::T3History native;
    ASSERT_EQ(t::InitializeLayeredJ2History(r,material,{},accepted),t::Status::kSuccess);
    ASSERT_EQ(nt::InitializeHistory(nr,{},native.shell),nt::Status::kSuccess);
    Path path{rotate,refinement};double plastic_work=0,preload_stress=0;
    for(;path.step<160*refinement;++path.step) {
      SCOPED_TRACE(path.step);
      const auto in=path.Interval(input);t::LayeredJ2ForceTrial actual;oracle::T3Trial expected;
      ASSERT_EQ(t::EvaluateLayeredJ2Force(r,material,accepted,in,actual),t::Status::kSuccess);
      ASSERT_EQ(oracle::Evaluate(nr,native,t3_port_test::Native(in),Law(rate),expected),nt::Status::kSuccess);
      t3_force_port_test::Agreement(r,in,actual.force,expected.shell,cv::Tolerance);
      Sections(actual.proposed_section,expected.points);
      Diagnostics(actual.section_diagnostics,expected.section,expected.shell.kinematics.area,
                  expected.shell.diagnostics.effective_thickness);
      ASSERT_FALSE(::testing::Test::HasFailure());
      plastic_work+=expected.section.plastic_work_increment_j;
      accepted={actual.force.proposed_history,actual.proposed_section};
      native={expected.shell.proposed_history,expected.points};
      if(path.step+1==64*refinement) {
        Yielded(accepted.section,rate);preload_stress=native.points[0][0];
        EXPECT_NE(native.shell.data().thickness,input.thickness);
      }
    }
    EXPECT_GT(plastic_work,1e-4);EXPECT_GT(std::abs(native.points[0][0]-preload_stress),1e7);
  }
}
} // namespace layered_j2_test::recurrence
