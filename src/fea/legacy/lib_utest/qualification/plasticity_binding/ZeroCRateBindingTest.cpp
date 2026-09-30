#include "PlasticityBindingFixture.h"

namespace plasticity_binding_test {
namespace mat=tl::material;
TEST(ShellPlasticityZeroCRateBinding, ExplicitZeroCPolicySurvivesBothFamilyParameterQueriesAndOwnedScope) {
  Fixture f;
  f.materials[0].rate={true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC};
  f.materials[1].rate=f.materials[0].rate;
  f.materials[1].curve_id=0;
  f.materials[1].hardening=mat::ShellPlasticityHardeningKind::LinearLaw44;
  f.materials[1].linear={5400,30000};
  auto input=f.catalog();
  input.curve_count=1;
  fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  Binding binding;
  ASSERT_EQ(binding.Initialize(native,input).status,Status::Success);
  Binding retained(binding);
  EXPECT_TRUE(binding.SameScope(retained));
  for(auto family:{fe::ShellBindingFamily::Qeph,fe::ShellBindingFamily::T3}) {
    fe::sections::PointParameters p;
    ASSERT_TRUE(retained.Parameters(family,0,&p));
    EXPECT_EQ(p.rate.policy,mat::ShellPlasticityRatePolicy::FilteredZeroC);
    EXPECT_EQ(p.inverse_rate_c,0.);
    EXPECT_EQ(p.inverse_rate_p,1.);
    EXPECT_GT(p.angular_cutoff_per_s,0.);
  }
  f.materials[1].rate.cutoff_hz=9000;
  Binding different;
  ASSERT_EQ(different.Initialize(native,input).status,Status::Success);
  EXPECT_FALSE(binding.SameScope(different));
  fe::sections::PointParameters original;
  ASSERT_TRUE(retained.Parameters(fe::ShellBindingFamily::T3,0,&original));
  EXPECT_EQ(original.rate.cutoff_hz,10000.);
}
TEST(ShellPlasticityZeroCRateBinding, LateWrongOrUnknownPolicyRejectsBeforePublicationThenRetries) {
  for(auto policy:{mat::ShellPlasticityRatePolicy::Legacy,
      static_cast<mat::ShellPlasticityRatePolicy>(99)}) {
    Fixture f;
    f.materials[1].rate={true,0,1,10000,policy};
    fe::ShellBatchBinding native;
    ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    Binding binding;
    const auto before=Bytes(binding);
    EXPECT_EQ(binding.Initialize(native,f.catalog()).status,Status::InvalidMaterial);
    EXPECT_EQ(Bytes(binding),before);
    f.materials[1].rate.policy=mat::ShellPlasticityRatePolicy::FilteredZeroC;
    EXPECT_EQ(binding.Initialize(native,f.catalog()).status,Status::Success);
  }
}
} // namespace plasticity_binding_test
