#include "PlacementNativeSupport.h"
namespace placement_test {
TEST(ShellPlacementNative, ExactNativeQuadratureAndFamilyCoefficientOperations) {
  constexpr double area=.003,thickness=.00228,mass=.1,length=.01;
  for(auto plane:Planes) {
    double position[3],force[3],moment[3],shift;
    placement_native_shift(NativeIpos(plane),&shift);
    placement_native_rule(NativeIpos(plane),position,force,moment);
    EXPECT_EQ(Bytes(shift),Bytes(tl::fea::NativeShellShift(plane)));
    for(unsigned p=0;p<3;++p) {
      EXPECT_EQ(Bytes(position[p]),Bytes(sec::LayerPosition(p,plane)));
      EXPECT_EQ(Bytes(force[p]),Bytes(sec::LayerForceWeight(p)));
      EXPECT_EQ(Bytes(moment[p]),Bytes(sec::LayerMomentWeight(p,plane)));
    }
    namespace q=tl::fea::qeph;
    namespace t=tl::fea::t3;
    q::ReferenceInput qi;qi.thickness=thickness;qi.density=2500;qi.young_modulus=70e9;qi.poisson_ratio=.22;
    t::ReferenceInput ti;ti.thickness=thickness;ti.density=2500;ti.young_modulus=70e9;ti.poisson_ratio=.22;
    q::detail::MaterialWork qm;
    t::detail::MaterialWork tm;
    ASSERT_TRUE(q::detail::PrepareMaterial(qi,area,1.e-7,qm,plane));
    ASSERT_TRUE(t::detail::PrepareMaterial(ti,area,tm,plane));
    q::detail::GeometryWork qg;qg.values.area=area;qg.values.characteristic_length=length;
    t::detail::GeometryWork tg;tg.kinematics.area=area;tg.kinematics.area_scale=1;
    tg.kinematics.characteristic_length=length;
    for(double active:{1.,0.}) {
      double native[10];
      placement_native_coefficients(shift,mass,area,thickness,70e9/(1-.22*.22),
          70e9/(2*(1+.22)),5./6.,length,qm.dm,std::sqrt(70e9/2500.),active,native);
      EXPECT_EQ(Bytes(native[0]),Bytes(tl::fea::NativeQephPlacementInertia(mass,area,thickness,plane)));
      EXPECT_EQ(Bytes(native[1]),Bytes(tl::fea::NativeT3PlacementInertia(mass,area,thickness)));
      EXPECT_EQ(Bytes(native[2]),Bytes(qm.offset));
      EXPECT_EQ(Bytes(native[2]),Bytes(tm.offset));
      q::ForceDiagnostics qd;t::ForceDiagnostics td;
      q::detail::StiffnessDiagnostics(qg,qm,qd,active);
      t::detail::StiffnessDiagnostics(tg,tm,td,active);
      Close(native[4],qd.translational_stiffness);
      Close(native[5],qd.rotational_stiffness);
      Close(native[6],td.translational_stiffness);
      Close(native[7],td.rotational_stiffness);
      EXPECT_EQ(Bytes(native[9]),Bytes(qd.unscaled_element_dt));
      EXPECT_EQ(Bytes(native[9]),Bytes(td.unscaled_element_dt));
    }
  }
}
TEST(ShellPlacementNative, CompleteGlassHistoryWorkRemovalAndTwoLaterIntervals) {
  const auto material=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  for(auto plane:Planes) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(NativeIpos(plane));
    SCOPED_TRACE(mask);
    auto history=Seed(mask);
    auto native=NativeSeed(history);
    Work work;work.thickness=.00228;native.thickness=work.thickness;
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32;++step) {
      SCOPED_TRACE(step);
      const auto input=Input(step,work.thickness,plane);
      NativeTrace trace;
      NativePlacementStep(input,(step+1)*input.dt,.001,.02,native,trace);
      sec::ShellLayeredTab1Result result;
      ASSERT_EQ(sec::UpdateShellLayeredTab1(material,failure,history,input,(step+1)*input.dt,result),PointStatus::Ok);
      ASSERT_TRUE(sec::ApplyLayeredTab1Work(result,input.strain_curvature_increment,
          input.reference_thickness,.001,trace.diagnostics[8],work,plane));
      Compare(result,work,native,trace,input.reference_thickness,.001);
      EXPECT_TRUE(sec::MatchesLayeredTab1Resultants(result.history,work,plane));
      for(unsigned p=0;p<3;++p)
        Close(result.caller_failure_increment[p],native.points[7*p+5]-history.saved.point[p].plastic_strain);
      removed+=result.removed_now;
      post+=!history.element_active;
      history=result.history;
    }
    EXPECT_EQ(removed,1u);EXPECT_GE(post,2u);
  }
}
TEST(ShellPlacementNative, CenteredNativePacketPreservesEveryExistingFieldBit) {
  for(unsigned mask=0;mask<8;++mask) {
    auto before=NativeSeed(Seed(mask));
    auto after=before;
    before.thickness=after.thickness=.00228;
    for(unsigned step=0;step<32;++step) {
      const auto input=Input(step,before.thickness,Placement::Centered);
      NativeTrace a,b;
      NativeStep(input,(step+1)*input.dt,.001,.02,before,a);
      NativePlacementStep(input,(step+1)*input.dt,.001,.02,after,b);
      SameNativeState(before,after);
      EXPECT_EQ(Bytes(a.point_values),Bytes(b.point_values));
      EXPECT_EQ(Bytes(a.diagnostics),Bytes(b.diagnostics));
      EXPECT_EQ(a.removed,b.removed);
    }
  }
}
} // namespace placement_test
