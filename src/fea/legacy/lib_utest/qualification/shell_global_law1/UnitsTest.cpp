#include "UnitComparison.h"
namespace global_law1_test {
TEST(GlobalLaw1Native,NativeCoefficientFloorMetreMillimetreAndCentimetreNeighbors) {
  EXPECT_EQ(global::NativeEm20(),1./1e20);
  auto qi=Quad();q::ReferenceData qr;ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
  auto ti=Triangle();t::ReferenceData tr;ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
  for(double length:{1.,.001,.01}) {
    const double floor=global::NativeEm20()*length;
    for(double h:{floor*.5,std::nextafter(floor,0.),floor,std::nextafter(floor,1.),floor*2.}) {
      auto qh=QHistory(qr,h);auto th=THistory(tr,h);q::ForceTrial a;t::ForceTrial b;
      auto qin=qeph_force_port_test::Next(qi,qh);auto tin=t3_port_test::Interval(ti,1e-6);tin.sample_index=1;
      ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(length),qr,qh,qin,a),q::Status::kSuccess);
      ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(length),tr,th,tin,b),t::Status::kSuccess);
      const double native_q=global::QephThickness(qi.thickness/length,h/length,1)*length;
      const double native_t=global::T3Thickness(ti.thickness/length,h/length,1)*length;
      // Native input/output conversion can round by one ULP. No broad force
      // tolerance is allowed to hide a wrong dimensioned coefficient floor.
      const double ulp=std::nextafter(h,std::numeric_limits<double>::infinity())-h;
      EXPECT_LE(std::abs(a.diagnostics.effective_thickness-native_q),2*ulp);
      EXPECT_LE(std::abs(b.diagnostics.effective_thickness-native_t),2*ulp);
      EXPECT_EQ(a.diagnostics.effective_thickness,std::max(floor,h));
      EXPECT_EQ(b.diagnostics.effective_thickness,h);
    }
    EXPECT_EQ(global::QephThickness(qi.thickness/length,floor/length,0),qi.thickness/length);
    EXPECT_EQ(global::T3Thickness(ti.thickness/length,floor/length,0),ti.thickness/length);
  }
}
TEST(GlobalLaw1Native,QephAcceptedThicknessSiMatchesMillimetreTonneNativeHistoryAndForces) {
  const Units u{.001,1000};auto input=Quad();input.projection_working_length_m=u.length;
  q::ReferenceData r;nq::Reference nr;
  ASSERT_EQ(q::InitializeReference(input,r),q::Status::kSuccess);
  ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(u.Reference(input)),nr),nq::Status::kSuccess);
  auto h=QHistory(r);nq::History n;ASSERT_EQ(nq::InitializeHistory(nr,{},n),nq::Status::kSuccess);
  path::Path motion{true,1};motion.angular_speed=18000;
  for(;motion.step<160;++motion.step) {
    SCOPED_TRACE(motion.step);const auto in=motion.Interval(input);q::ForceTrial a;nq::ForceTrial b;
    ASSERT_EQ(q::EvaluateGlobalLaw1Force(Accepted(u.length),r,h,in,a),q::Status::kSuccess);
    ASSERT_EQ(global::Evaluate(nr,n,u.Interval(in),1,b),nq::Status::kSuccess);
    CommonForce<4>(a,b,u,input.young_modulus,input.thickness);
    const auto& x=a.proposed_history.data();const auto& y=b.proposed_history.data();
    for(unsigned i=0;i<12;++i)UnitClose(x.stabilization[i],y.stabilization[i]*u.stress()/
      ((i==2||i==3||i==8||i==9)?u.length:1.),input.young_modulus*((i==2||i==3||i==8||i==9)?100.:1.));
    UnitClose(x.hourglass_viscous_work,y.hourglass_viscous_work*u.work(),input.young_modulus*input.thickness*.0004);
    UnitClose(a.diagnostics.hourglass_viscous_work_increment,b.diagnostics.hourglass_viscous_work_increment*u.work(),input.young_modulus*input.thickness*.0004);
    EXPECT_EQ(a.diagnostics.stabilization_viscosity,b.diagnostics.stabilization_viscosity);
    ASSERT_FALSE(::testing::Test::HasFailure());h=a.proposed_history;n=b.proposed_history;
  }
}
TEST(GlobalLaw1Native,T3AcceptedThicknessSiMatchesMillimetreTonneNativeHistoryAndForces) {
  const Units u{.001,1000};auto input=Triangle();t::ReferenceData r;nt::Reference nr;
  ASSERT_EQ(t::InitializeReference(input,r),t::Status::kSuccess);
  ASSERT_EQ(nt::Initialize(t3_port_test::Native(u.Reference(input)),nr),nt::Status::kSuccess);
  auto h=THistory(r);nt::History n;ASSERT_EQ(nt::InitializeHistory(nr,{},n),nt::Status::kSuccess);
  path::Path motion{true,1};motion.angular_speed=18000;
  for(;motion.step<160;++motion.step) {
    SCOPED_TRACE(motion.step);const auto in=motion.Interval(input);t::ForceTrial a;nt::ForceTrial b;
    ASSERT_EQ(t::EvaluateGlobalLaw1Force(Accepted(u.length),r,h,in,a),t::Status::kSuccess);
    ASSERT_EQ(global::Evaluate(nr,n,u.Interval(in),1,b),nt::Status::kSuccess);
    CommonForce<3>(a,b,u,input.young_modulus,input.thickness);
    UnitClose(a.proposed_history.data().equivalent_strain_rate,b.proposed_history.data().equivalent_strain_rate,1/in.dt);
    UnitClose(a.diagnostics.shear_factor,b.diagnostics.shear_factor,1.);
    UnitClose(a.diagnostics.transverse_shear_modulus,b.diagnostics.transverse_shear_modulus*u.stress(),input.young_modulus);
    ASSERT_FALSE(::testing::Test::HasFailure());h=a.proposed_history;n=b.proposed_history;
  }
}
} // namespace global_law1_test
