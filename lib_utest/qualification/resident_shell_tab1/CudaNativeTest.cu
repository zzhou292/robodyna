#include "CudaFixture.h"
#include "NativeAgreement.h"

namespace resident_tab1_test {
TEST_F(Cuda,CompleteResidentPoliciesBothPlanesNativeRemovalAndTwoInactiveIntervals) {
  constexpr unsigned last = Parents - 1;
  for (auto plane : placed::Planes) {
    Rig rig;
    ASSERT_TRUE(rig.Initialize(plane));
    ASSERT_TRUE(rig.Bind());
    ASSERT_EQ(rig.publication.Initialize(rig.owner, rig.qeph, rig.t3).status, fe::ShellPublicationStatus::Success);
    Frame old;
    ASSERT_TRUE(Read(rig, old));
    const auto qalloc = rig.qeph.allocations(), talloc = rig.t3.allocations();
    EXPECT_EQ(qalloc.device_allocations, 3u);
    EXPECT_EQ(talloc.device_allocations, 3u);
    const auto qr = rig.binding.qeph_reference(last);
    const auto tr = rig.binding.t3_reference(last);
    fe::sections::PointParameters material;
    ASSERT_TRUE(rig.catalog.Parameters(fe::ShellBindingFamily::Qeph, last, &material));
    const auto failure = rig.failure.parent(fe::ShellBindingFamily::Qeph, last)->tab1;
    auto qnative = native::Seed(q::LayeredTab1History{old.qforce[last].proposed_history,
        storage::Tab1FailureHistory(*old.qsection[last].plastic(), old.qfailure[last])});
    auto tnative = native::Seed(t::LayeredTab1History{old.tforce[last].proposed_history,
        storage::Tab1FailureHistory(*old.tsection[last].plastic(), old.tfailure[last])});
    unsigned qremoved = 0, tremoved = 0, post = 0;
    for (unsigned step = 0; step < 32 && post < 2; ++step) {
      Prepared prepared;
      ASSERT_TRUE(Prepare(rig, prepared));
      Frame candidate;
      ASSERT_TRUE(Evaluate(rig, prepared, candidate));
      const auto qi = Interval(qr, rig.binding.qeph_nodes(last), prepared);
      const auto ti = Interval(tr, rig.binding.t3_nodes(last), prepared);
      auto qtrial = qnative, ttrial = tnative;
      native::Advance(qr, qi, material, failure, qtrial);
      native::Advance(tr, ti, material, failure, ttrial);
      NativeAgreement<placed::Q>(candidate.qforce[last], *candidate.qsection[last].plastic(),
          candidate.qfailure[last], qtrial, qi, old.qforce[last].proposed_history.data().thickness,
          old.qsection[last].plastic()->cumulative_plastic_work_J);
      NativeAgreement<placed::T>(candidate.tforce[last], *candidate.tsection[last].plastic(),
          candidate.tfailure[last], ttrial, ti, old.tforce[last].proposed_history.data().thickness,
          old.tsection[last].plastic()->cumulative_plastic_work_J);
      qremoved += old.qfailure[last].active && !candidate.qfailure[last].active;
      tremoved += old.tfailure[last].active && !candidate.tfailure[last].active;
      post += !old.qfailure[last].active && !old.tfailure[last].active;
      Frame held;
      ASSERT_TRUE(Read(rig, held));
      Same(held, old);
      ASSERT_TRUE(Commit(rig, prepared, candidate));
      Frame accepted;
      ASSERT_TRUE(Read(rig, accepted));
      Same(accepted, candidate);
      EXPECT_EQ(rig.owner.accepted().epoch, step + 1);
      EXPECT_EQ(rig.qeph.allocations().device_bytes, qalloc.device_bytes);
      EXPECT_EQ(rig.t3.allocations().device_bytes, talloc.device_bytes);
      for (unsigned e = 0; e < 2; ++e) {
        EXPECT_TRUE(accepted.qfailure[e].active);
        EXPECT_TRUE(accepted.tfailure[e].active);
        EXPECT_EQ(accepted.qfailure[e].constant_points(), nullptr);
        EXPECT_EQ(accepted.qfailure[e].tab1_points(), nullptr);
      }
      old = accepted;
      qnative = qtrial;
      tnative = ttrial;
    }
    EXPECT_EQ(qremoved, 1u);
    EXPECT_EQ(tremoved, 1u);
    EXPECT_EQ(post, 2u);
  }
}
TEST_F(Cuda,OffsetAdmissionAndOtherFamilyParameterIdentityRemainComplete) {
  Rig omitted;
  ASSERT_TRUE(omitted.Initialize(Placement::TopReferencePlane, false, true));
  EXPECT_EQ(omitted.qeph.allocations().device_allocations, 0u);
  EXPECT_EQ(omitted.t3.allocations().device_allocations, 0u);
  Rig mismatch;
  ASSERT_TRUE(mismatch.Initialize(Placement::BottomReferencePlane, true));
  ASSERT_TRUE(mismatch.Bind());
  EXPECT_EQ(mismatch.publication.Initialize(mismatch.owner, mismatch.qeph, mismatch.t3).status,
            fe::ShellPublicationStatus::InvalidInput);
}
} // namespace resident_tab1_test
