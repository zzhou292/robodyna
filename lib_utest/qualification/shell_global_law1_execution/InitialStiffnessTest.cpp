#include "../shell_global_law1/UnitComparison.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
namespace global_law1_execution_initial_test {
using namespace global_law1_test;
using Law=tl::fea::ShellSectionLaw;
void Relative(double actual,double expected) {
  ASSERT_TRUE(std::isfinite(actual));ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected),2e-11*std::max(std::abs(actual),std::abs(expected)));
}
TEST(GlobalLaw1InitialStiffness,NativeVirginCoefficientFloorAtBothUnitContextsAndNeighborValues) {
  for(double length:{1.,.001})for(int ithk:{0,1}) {
    const Units units{length,length==1.?1.:1000.};
    const double floor=global::NativeEm20()*length;
    for(double thickness:{.5*floor,std::nextafter(floor,0.),floor,std::nextafter(floor,1.),2*floor}) {
      SCOPED_TRACE(length);
      SCOPED_TRACE(ithk);
      SCOPED_TRACE(thickness);
      Profile profile{ithk?Thickness::Accepted:Thickness::Reference,length};
      auto qi=Quad();qi.thickness=thickness;qi.projection_working_length_m=length;
      q::ReferenceData qr;nq::Reference qn;
      ASSERT_EQ(q::InitializeReference(qi,qr),q::Status::kSuccess);
      ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(units.Reference(qi)),qn),nq::Status::kSuccess);
      nq::History qh;ASSERT_EQ(nq::InitializeHistory(qn,{},qh),nq::Status::kSuccess);
      q::PrescribedInterval qp;qp.dt=1e-6;qp.sample_index=1;
      for(unsigned i=0;i<4;++i)qp.position_endpoint[i]=qi.position[i];
      nq::ForceTrial qforce;ASSERT_EQ(global::Evaluate(qn,qh,units.Interval(qp),ithk,qforce),nq::Status::kSuccess);
      q::mapped::NodalStiffness qactual;
      ASSERT_TRUE(q::mapped::InitialStiffness(qr,Law::GlobalLaw1Npt0,qactual,&profile));
      for(unsigned i=0;i<4;++i) {
        Relative(qactual.translation[i],qforce.diagnostics.translational_stiffness*units.mass*qforce.kinematics.nodal_factors[i%2]);
        Relative(qactual.rotation[i],qforce.diagnostics.rotational_stiffness*units.work()*qforce.kinematics.nodal_factors[i%2]);
      }
      const auto before=Bytes(qactual);
      EXPECT_FALSE(q::mapped::InitialStiffness(qr,Law::GlobalLaw1Npt0,qactual));EXPECT_EQ(Bytes(qactual),before);
      auto ti=Triangle();ti.thickness=thickness;t::ReferenceData tr;nt::Reference tn;
      ASSERT_EQ(t::InitializeReference(ti,tr),t::Status::kSuccess);
      ASSERT_EQ(nt::Initialize(t3_port_test::Native(units.Reference(ti)),tn),nt::Status::kSuccess);
      nt::History th;ASSERT_EQ(nt::InitializeHistory(tn,{},th),nt::Status::kSuccess);
      auto tp=t3_port_test::Interval(ti,1e-6);tp.sample_index=1;nt::ForceTrial tforce;
      ASSERT_EQ(global::Evaluate(tn,th,units.Interval(tp),ithk,tforce),nt::Status::kSuccess);
      t::mapped::NodalStiffness tactual;
      ASSERT_TRUE(t::mapped::InitialStiffness(tr,Law::GlobalLaw1Npt0,tactual,&profile));
      for(unsigned i=0;i<3;++i) {
        Relative(tactual.translation[i],tforce.diagnostics.translational_stiffness*units.mass);
        Relative(tactual.rotation[i],tforce.diagnostics.rotational_stiffness*units.work());
      }
    }
  }
}
} // namespace global_law1_execution_initial_test
