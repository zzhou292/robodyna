#include "SourcePartT3MaterialOracle.h"
#include "SourcePartT3ProjectionFixture.h"
#include "lib_utest/qualification/t3/T3ForcePortFixture.h"
#include <gtest/gtest-spi.h>

namespace crash::cases::source_part_elastic::test {
namespace {
namespace native=tl::qualification::t3;
namespace port=tl::fea::t3;
namespace adapter=t3_force_port_test;
TEST(SourcePartT3Material,CapturedProjectionResolutionKeepsBothStrictLayers) {
    const T3ProjectionResolutionFixture f;
    const auto reference=native::kinematic_test::MakeReference(f.reference);
    const auto base=native::force_test::MakeHistory(reference,f.base,{f.input.base_time,8421});
    native::ForceTrial result;
    ASSERT_EQ(native::EvaluateForce(reference,base,f.input,result),native::Status::kSuccess);
    const auto pr=adapter::Reference(adapter::Input(f.reference));
    const auto ph=adapter::History(pr,adapter::Values(f.base),{f.input.base_time,8421});
    port::ForceTrial p;
    ASSERT_EQ(port::EvaluateForce(pr,ph,adapter::Input(f.input),p),port::Status::kSuccess);
    adapter::Agreement(pr,adapter::Input(f.input),p,result);
    // Preserve the original failed full-chain evidence instead of silently
    // claiming its 2e-11 Pa bound passed. Native and port lose the same tiny
    // membrane rates when projecting unit translation in binary64.
    for(unsigned k=0;k<3;++k) { EXPECT_EQ(result.kinematics.raw_rate[k],0); EXPECT_EQ(p.kinematics.raw_rate[k],0); }
    const auto ideal=native::force_test::Independent(f.reference,f.base,f.input);
    const auto actual=result.proposed_history.data().stress[0];
    EXPECT_GT(std::abs(static_cast<long double>(actual)-ideal.values[0]),
        2e-11L+2e-10L*std::max(std::abs(static_cast<long double>(actual)),std::abs(ideal.values[0])));
    auto relative=f.input; const auto shift=relative.velocity[0];
    for(auto& v:relative.velocity) { v.x-=shift.x; v.y-=shift.y; v.z-=shift.z; }
    const auto centered=native::force_test::Independent(f.reference,f.base,relative);
    EXPECT_GT(centered.values[0],2e-11L);
    EXPECT_LT(std::abs(centered.values[0]-ideal.values[0]),1e-13L);
    t3_material::Check(reference,f.base,f.input,result);
    t3_material::Check(reference,f.base,f.input,adapter::Native(reference,p));
}
TEST(SourcePartT3Material,StrictMaterialCheckDetectsStressCorruption) {
    const T3ProjectionResolutionFixture f;
    const auto reference=native::kinematic_test::MakeReference(f.reference);
    const auto base=native::force_test::MakeHistory(reference,f.base,{f.input.base_time,8421});
    native::ForceTrial result;
    ASSERT_EQ(native::EvaluateForce(reference,base,f.input,result),native::Status::kSuccess);
    auto corrupt=result.proposed_history.data(); corrupt.material_stress[0]+=1;
    result.proposed_history=native::force_test::MakeHistory(reference,corrupt,result.proposed_history.stamp());
    ::testing::TestPartResultArray failures;
    { ::testing::ScopedFakeTestPartResultReporter capture(
        ::testing::ScopedFakeTestPartResultReporter::INTERCEPT_ONLY_CURRENT_THREAD,&failures);
      t3_material::Check(reference,f.base,f.input,result); }
    ASSERT_EQ(failures.size(),1); EXPECT_TRUE(failures.GetTestPartResult(0).failed());
}
} // namespace
} // namespace crash::cases::source_part_elastic::test
