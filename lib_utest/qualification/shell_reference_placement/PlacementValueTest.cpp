#include "PlacementFixture.h"
#include <limits>
namespace placement_test {
TEST(ShellPlacement, NativeNonzeroWeightsDifferFromGenericLeverArmAndPreserveCenter) {
  for(unsigned p=0;p<3;++p) {
    EXPECT_EQ(Bytes(sec::LayerPosition(p)),Bytes(sec::LayerPosition(p,Placement::Centered)));
    EXPECT_EQ(Bytes(sec::LayerMomentWeight(p)),Bytes(sec::LayerMomentWeight(p,Placement::Centered)));
    const double top[]{-1.,-.5,0.},bottom[]{0.,.5,1.};
    EXPECT_EQ(sec::LayerPosition(p,Placement::TopReferencePlane),top[p]);
    EXPECT_EQ(sec::LayerPosition(p,Placement::BottomReferencePlane),bottom[p]);
    const double factor=p==1?.5:(.5-(1./3.)*.5)*.5;
    EXPECT_EQ(sec::LayerMomentWeight(p,Placement::TopReferencePlane),top[p]*factor);
    EXPECT_EQ(sec::LayerMomentWeight(p,Placement::BottomReferencePlane),bottom[p]*factor);
  }
  EXPECT_NE(sec::LayerMomentWeight(0,Placement::TopReferencePlane),
      sec::LayerMomentWeight(0)-.5*sec::LayerForceWeight(0));
}
TEST(ShellPlacement, CoefficientsKeepNativeFamilyInertiaAsymmetryAndRejectAtomically) {
  constexpr double mass=.1,area=.002,thickness=.00228;
  const double centered=tl::fea::NativeQephPlacementInertia(mass,area,thickness,Placement::Centered);
  EXPECT_EQ(Bytes(centered),Bytes(mass*(area/12+thickness*thickness*(1./12+0))));
  const double triangle=tl::fea::NativeT3PlacementInertia(mass,area,thickness);
  EXPECT_EQ(Bytes(triangle),Bytes(mass*(area/(9./2)+thickness*thickness*(1./12))));
  for(auto p:Planes) {
    tl::fea::ShellPlacementCoefficients value;
    ASSERT_TRUE(tl::fea::PrepareShellPlacementCoefficients(p,thickness,value));
    EXPECT_EQ(value.stiffness_factor,p==Placement::Centered?1.:1.25);
    if(p!=Placement::Centered) {
      EXPECT_GT(tl::fea::NativeQephPlacementInertia(mass,area,thickness,p),centered);
      EXPECT_EQ(std::abs(value.offset),.5*thickness);
    }
  }
  tl::fea::ShellPlacementCoefficients sentinel{17,19};
  const auto before=Bytes(sentinel);
  EXPECT_FALSE(tl::fea::PrepareShellPlacementCoefficients(static_cast<Placement>(255),thickness,sentinel));
  EXPECT_EQ(before,Bytes(sentinel));
  EXPECT_FALSE(tl::fea::PrepareShellPlacementCoefficients(Placement::Centered,0,sentinel));
  EXPECT_EQ(before,Bytes(sentinel));
}
TEST(ShellPlacement, OriginalGlassSectionShiftChangesResponseAndKeepsCheckedWork) {
  const auto material=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  double moment[3]{};
  for(unsigned plane=0;plane<3;++plane) {
    auto input=Input(0,.00228,Planes[plane]);
    sec::ShellLayeredTab1Result result;
    ASSERT_EQ(sec::UpdateShellLayeredTab1(material,failure,Seed(0),input,input.dt,result),PointStatus::Ok);
    Work work; work.thickness=.00228;
    ASSERT_TRUE(sec::ApplyLayeredTab1Work(result,input.strain_curvature_increment,
        input.reference_thickness,.001,Viscosity(input,.001,.02),work,input.placement));
    EXPECT_TRUE(sec::MatchesLayeredTab1Resultants(result.history,work,input.placement));
    moment[plane]=result.current.bending_stress[0];
    if(plane) EXPECT_FALSE(sec::MatchesLayeredTab1Resultants(result.history,work));
  }
  EXPECT_LT(moment[1],moment[0]);
  EXPECT_GT(moment[2],moment[0]);
}
TEST(ShellPlacement, InvalidPlaneAndLatePointFailurePreserveOutputAndExactRetry) {
  const auto material=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  auto input=Input(0,.00228,Placement::TopReferencePlane);
  const auto clean=Seed(0);
  sec::ShellLayeredTab1Result expected,actual;
  ASSERT_EQ(sec::UpdateShellLayeredTab1(material,failure,clean,input,input.dt,expected),PointStatus::Ok);
  const auto before=Bytes(actual);
  input.placement=static_cast<Placement>(255);
  EXPECT_EQ(sec::UpdateShellLayeredTab1(material,failure,clean,input,input.dt,actual),PointStatus::InvalidIncrement);
  EXPECT_EQ(before,Bytes(actual));
  input.placement=Placement::TopReferencePlane;
  auto bad=clean;
  bad.saved.point[2].filtered_rate_per_s=std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(sec::UpdateShellLayeredTab1(material,failure,bad,input,input.dt,actual),PointStatus::Ok);
  EXPECT_EQ(before,Bytes(actual));
  ASSERT_EQ(sec::UpdateShellLayeredTab1(material,failure,clean,input,input.dt,actual),PointStatus::Ok);
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c)
      EXPECT_EQ(Bytes(actual.history.current_force_point[p].stress[c]),Bytes(expected.history.current_force_point[p].stress[c]));
    SameFailureHistory(actual.history.failure[p],expected.history.failure[p]);
  }
  Work work;
  const auto old_work=Bytes(work);
  EXPECT_FALSE(sec::ApplyLayeredTab1Work(actual,input.strain_curvature_increment,
      input.reference_thickness,.001,0,work,static_cast<Placement>(255)));
  EXPECT_EQ(old_work,Bytes(work));
}
} // namespace placement_test
