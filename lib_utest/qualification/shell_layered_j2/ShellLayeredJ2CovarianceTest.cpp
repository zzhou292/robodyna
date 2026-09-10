#include "LayeredJ2CovarianceFixture.h"

namespace layered_j2_test {
namespace cv=covariance;

TEST(ShellLayeredJ2, YieldedQephHistoryHasWorldRotationAndTranslationCovariance) {
  const auto p=cv::source::Prepare();
  // Existing planar, warped and skewed source-scale QEPH geometries.
  for(unsigned shape:{4u,5u,8u}) {
    SCOPED_TRACE(shape);
    auto input=qeph_startup_test::Case(shape); cv::SourceMaterial(input,p);
    q::ReferenceData reference,rotated;
    ASSERT_EQ(q::InitializeReference(input,reference),q::Status::kSuccess);
    ASSERT_EQ(q::InitializeReference(qeph_startup_test::Reparameterize(input,0,true),rotated),q::Status::kSuccess);
    q::LayeredJ2History base,transformed;
    ASSERT_EQ(q::InitializeLayeredJ2History(reference,p,{},base),q::Status::kSuccess);
    cv::Path path;
    for(unsigned step=0;step<cv::PreloadSteps;++step) {
      q::LayeredJ2ForceTrial next;
      ASSERT_EQ(q::EvaluateLayeredJ2Force(reference,p,base,path.Interval(input,1.),next),q::Status::kSuccess);
      base={next.force.proposed_history,next.proposed_section}; path.Advance(1.);
    }
    cv::Yielded(base.section); ASSERT_FALSE(::testing::Test::HasFailure());
    EXPECT_NE(base.shell.data().thickness,input.thickness);
    // Local material/history components are unchanged by a superposed world
    // transform; bind the actual yielded values to the transformed reference.
    transformed.section=base.section;
    ASSERT_EQ(q::PreparePrescribedHistory(rotated,base.shell.data(),base.shell.stamp(),transformed.shell),q::Status::kSuccess);
    const double length=qeph_startup_test::Scale(input);
    for(unsigned step=0;step<96;++step) {
      SCOPED_TRACE(step);
      const double rate=cv::ContinuationRate(step);
      const auto in=path.Interval(input,rate);
      q::LayeredJ2ForceTrial a,b;
      ASSERT_EQ(q::EvaluateLayeredJ2Force(reference,p,base,in,a),q::Status::kSuccess);
      ASSERT_EQ(q::EvaluateLayeredJ2Force(rotated,p,transformed,
          qeph_kinematics_test::Reparameterize(in,0,true),b),q::Status::kSuccess);
      qeph_force_port_test::HistoryAgreement(b.force.proposed_history.data(),a.force.proposed_history.data(),
          input,length,cv::Tolerance);
      cv::Sections(b.proposed_section,a.proposed_section,p);
      cv::SectionDiagnostics(b.section_diagnostics,a.section_diagnostics,p);
      cv::Loads<4>(b.force,a.force,p,length);
      EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(a.proposed_section,a.force.proposed_history.data()));
      EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(b.proposed_section,b.force.proposed_history.data()));
      EXPECT_TRUE(a.force.proposed_history.matches_reference(reference));
      EXPECT_TRUE(b.force.proposed_history.matches_reference(rotated));
      ASSERT_FALSE(::testing::Test::HasFailure());
      base={a.force.proposed_history,a.proposed_section};
      transformed={b.force.proposed_history,b.proposed_section}; path.Advance(rate);
    }
    EXPECT_GT(base.section.point[0].plastic_strain+base.section.point[2].plastic_strain,1.e-3);
  }
}

TEST(ShellLayeredJ2, YieldedT3HistoryHasWorldRotationAndTranslationCovariance) {
  const auto p=cv::source::Prepare();
  for(unsigned shape:{0u,1u,2u}) {
    SCOPED_TRACE(shape);
    auto input=t3_port_test::Triangle(.02,shape); cv::SourceMaterial(input,p);
    t::ReferenceData reference,rotated;
    ASSERT_EQ(t::InitializeReference(input,reference),t::Status::kSuccess);
    ASSERT_EQ(t::InitializeReference(cv::Transform(input),rotated),t::Status::kSuccess);
    t::LayeredJ2History base,transformed;
    ASSERT_EQ(t::InitializeLayeredJ2History(reference,p,{},base),t::Status::kSuccess);
    cv::Path path;
    for(unsigned step=0;step<cv::PreloadSteps;++step) {
      t::LayeredJ2ForceTrial next;
      ASSERT_EQ(t::EvaluateLayeredJ2Force(reference,p,base,path.Interval(input,1.),next),t::Status::kSuccess);
      base={next.force.proposed_history,next.proposed_section}; path.Advance(1.);
    }
    cv::Yielded(base.section); ASSERT_FALSE(::testing::Test::HasFailure());
    EXPECT_NE(base.shell.data().thickness,input.thickness);
    transformed.section=base.section;
    ASSERT_EQ(t::PreparePrescribedHistory(rotated,base.shell.data(),base.shell.stamp(),transformed.shell),t::Status::kSuccess);
    double length=0;
    for(unsigned n=1;n<3;++n)
      length=std::max(length,cv::frame::Length(cv::frame::Difference(input.position[n],input.position[0])));
    for(unsigned step=0;step<96;++step) {
      SCOPED_TRACE(step);
      const double rate=cv::ContinuationRate(step);
      const auto in=path.Interval(input,rate);
      t::LayeredJ2ForceTrial a,b;
      ASSERT_EQ(t::EvaluateLayeredJ2Force(reference,p,base,in,a),t::Status::kSuccess);
      ASSERT_EQ(t::EvaluateLayeredJ2Force(rotated,p,transformed,cv::Transform(in),b),t::Status::kSuccess);
      cv::OrdinaryHistory(b.force.proposed_history.data(),a.force.proposed_history.data(),p,length);
      cv::Field(b.force.proposed_history.data().equivalent_strain_rate,
          a.force.proposed_history.data().equivalent_strain_rate,1./cv::Dt,cv::Tolerance,"total_strain_rate");
      cv::Sections(b.proposed_section,a.proposed_section,p);
      cv::SectionDiagnostics(b.section_diagnostics,a.section_diagnostics,p);
      cv::Loads<3>(b.force,a.force,p,length);
      EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(a.proposed_section,a.force.proposed_history.data()));
      EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(b.proposed_section,b.force.proposed_history.data()));
      EXPECT_TRUE(a.force.proposed_history.matches_reference(reference));
      EXPECT_TRUE(b.force.proposed_history.matches_reference(rotated));
      ASSERT_FALSE(::testing::Test::HasFailure());
      base={a.force.proposed_history,a.proposed_section};
      transformed={b.force.proposed_history,b.proposed_section}; path.Advance(rate);
    }
    EXPECT_GT(base.section.point[0].plastic_strain+base.section.point[2].plastic_strain,1.e-3);
  }
}
} // namespace layered_j2_test
