#include "native/NativePacket.h"
#include "../shell_tab1_force/NativeAgreement.h"

namespace placement_force_test {
template<class F> void NativeRecurrence() {
  for(auto plane:Planes) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(static_cast<unsigned>(plane));
    SCOPED_TRACE(mask);
    Fixture<F> f(plane,mask);
    auto packet=native::Seed(f.accepted);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32&&post<2;++step) {
      SCOPED_TRACE(step);
      const auto before=f.accepted;
      const auto interval=Interval(f.reference,step);
      typename F::Trial trial;
      ASSERT_EQ(f.Evaluate(step,trial),F::Status::kSuccess);
      native::Advance(f.reference,interval,f.material,f.failure,packet);
      base::Agreement<F>(trial,packet,interval,before.shell.data().thickness);
      removed+=trial.section.removed_now;
      if(!before.section.element_active) ++post;
      f.Accept(trial);
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(PlacementForceNative,QephOriginalGlassBothSignsAllMasksCompleteNativeForce) {NativeRecurrence<Q>();}
TEST(PlacementForceNative,T3OriginalGlassBothSignsAllMasksCompleteNativeForce) {NativeRecurrence<T>();}

template<class F> void CenteredNativeParity() {
  using NativeReference=std::conditional_t<std::is_same_v<F,Q>,base::native::nq::Reference,base::native::nt::Reference>;
  Fixture<F> f(Placement::Centered);
  const auto reference=base::native::Reference<NativeReference>(f.reference.input);
  auto original=native::Seed(f.accepted),placed=original;
  for(unsigned step=0;step<24;++step) {
    const auto interval=Interval(f.reference,step);
    base::native::Advance(reference,interval,f.material,f.failure,original);
    native::Advance(f.reference,interval,f.material,f.failure,placed);
    EXPECT_EQ(tab1_test::Bytes(original.history),tab1_test::Bytes(placed.history));
    EXPECT_EQ(tab1_test::Bytes(original.points),tab1_test::Bytes(placed.points));
    EXPECT_EQ(tab1_test::Bytes(original.failures),tab1_test::Bytes(placed.failures));
    EXPECT_EQ(tab1_test::Bytes(original.force),tab1_test::Bytes(placed.force));
    EXPECT_EQ(tab1_test::Bytes(original.point_values),tab1_test::Bytes(placed.point_values));
    EXPECT_EQ(tab1_test::Bytes(original.diagnostics),tab1_test::Bytes(placed.diagnostics));
    EXPECT_EQ(original.removed,placed.removed);
    EXPECT_EQ(original.planar,placed.planar);
  }
}
TEST(PlacementForceNative,ExplicitCenteredPacketPreservesAllOriginalQAndTNativeFields) {
  CenteredNativeParity<Q>();
  CenteredNativeParity<T>();
}
} // namespace placement_force_test
