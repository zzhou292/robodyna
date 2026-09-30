#include "T3PortFixture.h"
namespace t3_port_test {
TEST(T3StartupPort, ActualThreeNodeShapesScalesAndOrderingMatchNativeAndAnalyticMasses) {
  for(double scale:{1.,.02,1e-5}) for(unsigned shape=0;shape<3;++shape)
    for(unsigned ordering=0;ordering<6;++ordering) {
      auto r=Triangle(scale,shape); const auto saved=r; const bool reverse=ordering>=3;
      for(unsigned n=0;n<3;++n) { const unsigned i=(ordering%3+(reverse?3-n:n))%3;
        r.position[n]=saved.position[i]; r.node_ids[n]=saved.node_ids[i]; }
      r.node_ids[0]+=1ULL<<48;
      ASSERT_NO_FATAL_FAILURE(CheckStartup(r));
    }
}
TEST(T3StartupPort, RigidFramesEdgeOnAndSharedMassUseNativeAngleContributionsOnce) {
  auto first=Triangle(.02),second=first;
  second.position[2]={.012,-.018,0}; second.node_ids[2]=400;
  const auto a=Reference(first),b=Reference(second);
  const auto wa=native::test::Independent(Native(first)),wb=native::test::Independent(Native(second));
  for(unsigned n=0;n<2;++n) native::test::Near(a.nodal_mass[n]+b.nodal_mass[n],
      wa.mass*wa.weight[n]+wb.mass*wb.weight[n],wa.mass+wb.mass);
  for(unsigned variant=0;variant<2;++variant) {
    auto r=first;
    for(auto& x:r.position) {
      x=variant?kt::Rotate(kt::Rotation(),x):port::Vec3{x.y,x.z,x.x};
      x.x+=.125; x.y-=.25; x.z+=.5;
    }
    ASSERT_NO_FATAL_FAILURE(CheckStartup(r));
    const auto p=Reference(r);
    for(unsigned n=0;n<3;++n) Near(p.nodal_mass[n],a.nodal_mass[n],a.nodal_mass[n]);
  }
}
TEST(T3StartupPort, SectionAndDensityScalingRetainSeparateNativeInertiaExpressions) {
  auto r=Triangle(.02); const auto base=Reference(r);
  r.density*=8; const auto dense=Reference(r);
  r.density/=8; r.thickness*=4; const auto thick=Reference(r);
  for(unsigned n=0;n<3;++n) {
    Near(dense.nodal_mass[n],8*base.nodal_mass[n],dense.nodal_mass[n]);
    Near(dense.isotropic_inertia[n],8*base.isotropic_inertia[n],dense.isotropic_inertia[n]);
    Near(thick.nodal_mass[n],4*base.nodal_mass[n],thick.nodal_mass[n]);
    Near(thick.physical_inertia[n],64*base.physical_inertia[n],thick.physical_inertia[n]);
    Near(thick.added_inertia[n],4*base.added_inertia[n],thick.added_inertia[n]);
  }
  ASSERT_NO_FATAL_FAILURE(CheckStartup(r));
}
TEST(T3StartupPort, CutoffBandsRejectAmbiguityWithoutClaimingNativeAcceptanceParity) {
  const auto good=Triangle(); auto out=Reference(good); const auto unchanged=Bytes(out);
  const double b=port::detail::GuardBand;
  // Right-angle shapes keep ACOS far from its cutoff while isolating edge range.
  for(double factor:{.5,1.,1+.5*b}) {
    auto r=Triangle(port::detail::MinimumEdge*factor,1);
    EXPECT_EQ(port::InitializeReference(r,out),port::Status::kUnsupportedGeometry); EXPECT_EQ(Bytes(out),unchanged);
  }
  ASSERT_NO_FATAL_FAILURE(CheckStartup(Triangle(2*port::detail::MinimumEdge,1)));
  // The shared numerical geometry predicate separates a unit-scale area band
  // from startup ACOS admission. It does not alter its output on rejection.
  for(double height:{.5e-6,1e-6,1e-6+.25*b}) {
    const port::Vec3 x[3]={{0,0,0},{1,0,0},{.5,height,0}}; double longest=17;
    EXPECT_FALSE(port::detail::SupportedGeometry(x,longest)); EXPECT_EQ(longest,17);
  }
  const port::Vec3 safe[3]={{0,0,0},{1,0,0},{.5,2e-6,0}}; double longest=0;
  EXPECT_TRUE(port::detail::SupportedGeometry(safe,longest)); EXPECT_EQ(longest,1.);
  // Near-right triangles isolate the actual ACOS argument from the looser
  // normalized-area cutoff; only admission is asserted in this conditioning band.
  for(double margin:{.99*port::detail::AcosMargin,port::detail::AcosMargin+.25*b}) {
    auto r=Triangle(); r.position[1]={1,0,0}; r.position[2]={0,std::sqrt(2*margin),0};
    double length=0; ASSERT_TRUE(port::detail::SupportedGeometry(r.position,length));
    EXPECT_EQ(port::InitializeReference(r,out),port::Status::kUnsupportedGeometry); EXPECT_EQ(Bytes(out),unchanged);
  }
  auto admitted=Triangle(); admitted.position[1]={1,0,0};
  admitted.position[2]={0,std::sqrt(2*(port::detail::AcosMargin+4*b)),0};
  port::ReferenceData boundary_result;
  ASSERT_EQ(port::InitializeReference(admitted,boundary_result),port::Status::kSuccess);
  for(double scale:{1000/std::sqrt(2.),1001/std::sqrt(2.)}) {
    auto r=Triangle(scale,1); EXPECT_EQ(port::InitializeReference(r,out),port::Status::kUnsupportedGeometry);
    EXPECT_EQ(Bytes(out),unchanged);
  }
}
TEST(T3StartupPort, MalformedAndLateUnrepresentableMassPreserveOutputThenCleanAliasRetry) {
  const auto good=Triangle(.02); auto out=Reference(good); const auto saved=Bytes(out);
  for(unsigned fault=0;fault<7;++fault) {
    auto bad=good;
    if(fault==0)bad.node_ids[1]=bad.node_ids[0];
    if(fault==1)bad.position[2].z=std::numeric_limits<double>::quiet_NaN();
    if(fault==2)bad.density=-1;
    if(fault==3)bad.poisson_ratio=.5;
    if(fault==4)bad.young_modulus=0;
    if(fault==5)bad.thickness=std::numeric_limits<double>::denorm_min();
    if(fault==6)bad.density=std::numeric_limits<double>::max();
    // Force definite overflow in the final case while keeping valid geometry.
    if(fault==6)bad.thickness=100;
    EXPECT_NE(port::InitializeReference(bad,out),port::Status::kSuccess); EXPECT_EQ(Bytes(out),saved);
  }
  ASSERT_EQ(port::InitializeReference(out.input,out),port::Status::kSuccess);
  native::Reference nr; ASSERT_EQ(native::Initialize(Native(good),nr),native::Status::kSuccess);
  StartupAgreement(out,nr);
}
} // namespace t3_port_test
