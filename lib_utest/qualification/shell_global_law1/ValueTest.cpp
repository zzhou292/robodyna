#include "Fixture.h"
namespace global_law1_test {
TEST(GlobalLaw1Values,LegacyDefaultKeepsBothFamilyResultsAndHistory) {
  auto qi=Quad();q::ReferenceData qr;ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
  auto qh=QHistory(qr,.7*qi.thickness);path::Path motion{true,1};q::ForceTrial qa,qb;
  for(unsigned k=0;k<16;++k,++motion.step) {
    const auto in=motion.Interval(qi);
    ASSERT_EQ(q::EvaluateForce(qr,qh,in,qa),q::Status::kSuccess);
    ASSERT_EQ(q::EvaluateGlobalLaw1Force({},qr,qh,in,qb),q::Status::kSuccess);
    qeph_force_port_test::ForceAgreement(qa,qb,qi,in,0.);ForceBits(qa,qb);qh=qa.proposed_history;
  }
  auto ti=Triangle();t::ReferenceData tr;ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
  auto th=THistory(tr,.7*ti.thickness);motion={true,1};t::ForceTrial ta,tb;
  for(unsigned k=0;k<16;++k,++motion.step) {
    const auto in=motion.Interval(ti);
    ASSERT_EQ(t::EvaluateForce(tr,th,in,ta),t::Status::kSuccess);
    ASSERT_EQ(t::EvaluateGlobalLaw1Force({},tr,th,in,tb),t::Status::kSuccess);
    t3_force_port_test::Exact(ta,tb);ForceBits(ta,tb);th=ta.proposed_history;
  }
}
TEST(GlobalLaw1Values,AcceptedThicknessIsPriorForceThicknessNotNewReportedThickness) {
  auto qi=Quad();q::ReferenceData qr;ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
  const auto qh=QHistory(qr,.6*qi.thickness);path::Path motion;q::ForceTrial qa;
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(.001),qr,qh,motion.Interval(qi),qa),q::Status::kSuccess);
  EXPECT_EQ(qa.diagnostics.effective_thickness,qh.data().thickness);
  EXPECT_NE(qa.proposed_history.data().thickness,qh.data().thickness);
  auto ti=Triangle();t::ReferenceData tr;ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
  const auto th=THistory(tr,.6*ti.thickness);t::ForceTrial ta;
  ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(.001),tr,th,motion.Interval(ti),ta),t::Status::kSuccess);
  EXPECT_EQ(ta.diagnostics.effective_thickness,th.data().thickness);
  EXPECT_NE(ta.proposed_history.data().thickness,th.data().thickness);
}
TEST(GlobalLaw1Values,UnknownPolicyInvalidUnitAndWrongPhasePreserveOutputsAndRetry) {
  auto qi=Quad();q::ReferenceData qr;ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
  auto ti=Triangle();t::ReferenceData tr;ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
  const auto qh=QHistory(qr);const auto th=THistory(tr);path::Path motion;q::ForceTrial qa;t::ForceTrial ta;
  const auto qin=motion.Interval(qi);const auto tin=motion.Interval(ti);
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qh,qin,qa),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,th,tin,ta),t::Status::kSuccess);
  const auto qb=Bytes(qa);const auto tb=Bytes(ta);const auto qh0=Bytes(qh);const auto th0=Bytes(th);
  const Profile invalid[]{{static_cast<Thickness>(77),1.},{Thickness::Accepted,0.},
    {Thickness::Reference,-1.},{Thickness::Accepted,std::numeric_limits<double>::infinity()},
    {Thickness::Accepted,std::numeric_limits<double>::quiet_NaN()},{Thickness::Accepted,1e-310}};
  for(const auto& p:invalid) {
    EXPECT_EQ(q::EvaluateGlobalLaw1Force(p,qr,qh,qin,qa),q::Status::kInvalidInput);
    EXPECT_EQ(t::EvaluateGlobalLaw1Force(p,tr,th,tin,ta),t::Status::kInvalidInput);
    EXPECT_EQ(Bytes(qa),qb);EXPECT_EQ(Bytes(ta),tb);
  }
  auto qbad=qin;auto tbad=tin;++qbad.sample_index;++tbad.sample_index;
  EXPECT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qh,qbad,qa),q::Status::kInvalidInput);
  EXPECT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,th,tbad,ta),t::Status::kInvalidInput);
  EXPECT_EQ(Bytes(qa),qb);EXPECT_EQ(Bytes(ta),tb);EXPECT_EQ(Bytes(qh),qh0);EXPECT_EQ(Bytes(th),th0);
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qh,qin,qa),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,th,tin,ta),t::Status::kSuccess);
  EXPECT_EQ(Bytes(qa),qb);EXPECT_EQ(Bytes(ta),tb);
}
TEST(GlobalLaw1Values,LateCoefficientArithmeticFailurePreservesBothHistoriesAndResults) {
  const auto qi=Quad();q::ReferenceData qr;ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
  const auto ti=Triangle();t::ReferenceData tr;ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
  const auto qh=QHistory(qr);const auto th=THistory(tr);
  const auto qhuge=QHistory(qr,1e200);const auto thuge=THistory(tr,1e200);
  path::Path motion;q::ForceTrial qa;t::ForceTrial ta;
  const auto qin=motion.Interval(qi);const auto tin=motion.Interval(ti);
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qh,qin,qa),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,th,tin,ta),t::Status::kSuccess);
  const auto qb=Bytes(qa);const auto tb=Bytes(ta);const auto qbefore=Bytes(qhuge);const auto tbefore=Bytes(thuge);
  EXPECT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qhuge,qin,qa),q::Status::kNonfiniteResult);
  EXPECT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,thuge,tin,ta),t::Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(qa),qb);EXPECT_EQ(Bytes(ta),tb);EXPECT_EQ(Bytes(qhuge),qbefore);EXPECT_EQ(Bytes(thuge),tbefore);
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(),qr,qh,qin,qa),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(),tr,th,tin,ta),t::Status::kSuccess);
}
} // namespace global_law1_test
