#include "PlacementForceFixture.h"
#include "lib_src/elements/qeph/QephLayeredJ2Failure.h"
#include "lib_src/elements/t3/T3LayeredJ2Failure.h"
#include "lib_src/elements/qeph/QephLayeredLaw1.h"
#include "lib_src/elements/t3/T3LayeredLaw1.h"

namespace placement_force_test {
template<class F> void Recurrence() {
  for(auto plane:Planes) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(static_cast<unsigned>(plane));
    SCOPED_TRACE(mask);
    Fixture<F> f(plane,mask);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32&&post<2;++step) {
      SCOPED_TRACE(step);
      const auto before=f.accepted;
      typename F::Trial trial;
      ASSERT_EQ(f.Evaluate(step,trial),F::Status::kSuccess);
      EXPECT_TRUE(trial.force.proposed_history.matches_reference(f.reference));
      EXPECT_EQ(trial.force.proposed_history.stamp().sample_index,step+1u);
      removed+=trial.section.removed_now;
      if(!before.section.element_active) ++post;
      // A mismatched plane cannot borrow saved histories, even though top and
      // bottom native masses/inertias are numerically identical.
      auto wrong=f.reference;
      auto input=wrong.input;
      input.placement=plane==Placement::TopReferencePlane?Placement::BottomReferencePlane:Placement::TopReferencePlane;
      ASSERT_EQ(InitializeReference(input,wrong),F::Status::kSuccess);
      const auto previous=tab1_test::Bytes(trial);
      EXPECT_NE(EvaluateLayeredTab1Force(wrong,f.material,f.failure,f.accepted,
          Interval(wrong,step),trial),F::Status::kSuccess);
      EXPECT_EQ(tab1_test::Bytes(trial),previous);
      auto bad=f.accepted;
      bad.section.saved.point[2].plastic_strain=-1;
      EXPECT_NE(EvaluateLayeredTab1Force(f.reference,f.material,f.failure,bad,
          Interval(f.reference,step),trial),F::Status::kSuccess);
      EXPECT_EQ(tab1_test::Bytes(trial),previous);
      typename F::Trial retry;
      ASSERT_EQ(f.Evaluate(step,retry),F::Status::kSuccess);
      Exact(ForceValues(trial.force),ForceValues(retry.force));
      Exact(SectionValues(trial.section),SectionValues(retry.section));
      f.Accept(trial);
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(PlacementForceValues,QephBothPlacementSignsAndCenteredRemovalAtomicRetry) {Recurrence<Q>();}
TEST(PlacementForceValues,T3BothPlacementSignsAndCenteredRemovalAtomicRetry) {Recurrence<T>();}

TEST(PlacementForceValues,UnqualifiedForceFormulationsRejectNoncenteredReferences) {
  namespace q=tl::fea::qeph;
  namespace t=tl::fea::t3;
  Fixture<Q> quad(Placement::TopReferencePlane);
  Fixture<T> tri(Placement::BottomReferencePlane);
  q::LayeredJ2History qj;
  t::LayeredJ2History tj;
  EXPECT_EQ(q::InitializeLayeredJ2History(quad.reference,quad.material,{},qj),q::Status::kInvalidInput);
  EXPECT_EQ(t::InitializeLayeredJ2History(tri.reference,tri.material,{},tj),t::Status::kInvalidInput);
  q::LayeredJ2FailureHistory qd;
  t::LayeredJ2FailureHistory td;
  EXPECT_EQ(q::InitializeLayeredJ2FailureHistory(quad.reference,quad.material,{.015},{},qd),q::Status::kInvalidInput);
  EXPECT_EQ(t::InitializeLayeredJ2FailureHistory(tri.reference,tri.material,{.015},{},td),t::Status::kInvalidInput);
  tl::material::ShellElasticLaw1PointParameters elastic;
  ASSERT_TRUE(tl::material::PrepareShellElasticLaw1Point(70e9,.22,2500,elastic));
  q::LayeredLaw1History qe;
  t::LayeredLaw1History te;
  EXPECT_EQ(q::InitializeLayeredLaw1History(quad.reference,elastic,{},qe),q::Status::kInvalidInput);
  EXPECT_EQ(t::InitializeLayeredLaw1History(tri.reference,elastic,{},te),t::Status::kInvalidInput);
  q::ForceTrial qf;
  t::ForceTrial tf;
  EXPECT_EQ(q::EvaluateForce(quad.reference,quad.accepted.shell,Interval(quad.reference,0),qf),q::Status::kInvalidInput);
  EXPECT_EQ(t::EvaluateForce(tri.reference,tri.accepted.shell,Interval(tri.reference,0),tf),t::Status::kInvalidInput);
}
} // namespace placement_force_test
