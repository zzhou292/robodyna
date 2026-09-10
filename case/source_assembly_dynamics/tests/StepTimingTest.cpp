#include "Fixture.h"

namespace crash::cases::source_assembly_dynamics::test {
TEST_F(SourceAssemblyDynamicsCheck, OptionalTimingPreservesActualFieldsHistoriesAndAllocationAcrossContact) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);SourceAssemblyWallCase plain,timed;
    ASSERT_TRUE(plain.Initialize(bindings,setup,SmokeConfig()));
    ASSERT_TRUE(timed.Initialize(bindings,setup,SmokeConfig(),{true}));
    const auto allocations=plain.allocations();const auto host=plain.host_payload_bytes();
    SameAllocations(timed,allocations,host);
    constexpr unsigned steps=64;
    for(unsigned step=0;step<steps;++step) {
        const auto a=plain.Step(),b=timed.Step();ASSERT_TRUE(a)<<a.message;ASSERT_TRUE(b)<<b.message;
        const auto& p=SourceAssemblyDynamicsTestAccess::Accepted(plain);
        const auto& t=SourceAssemblyDynamicsTestAccess::Accepted(timed);
        SameFields(p.fields,t.fields);SameParents(p.parents,t.parents);
        EXPECT_EQ(p.diagnostics.maximum_plastic_strain,t.diagnostics.maximum_plastic_strain);
        EXPECT_EQ(p.diagnostics.cumulative_plastic_work,t.diagnostics.cumulative_plastic_work);
        EXPECT_EQ(p.diagnostics.motion.native_residual,t.diagnostics.motion.native_residual);
        EXPECT_EQ(p.wall.diagnostics.resultant.value,t.wall.diagnostics.resultant.value);
        SameAllocations(plain,allocations,host);SameAllocations(timed,allocations,host);
    }
    ASSERT_GT(timed.accepted_contact().diagnostics->resultant.value,0);
    const auto off=plain.timing(),on=timed.timing();EXPECT_FALSE(off.enabled);EXPECT_TRUE(on.enabled);
    EXPECT_EQ(on.clock_failures,0u);EXPECT_EQ(on.backward_samples,0u);EXPECT_FALSE(on.counter_saturated);
    std::uint64_t stage_sum=0;
    for(std::size_t i=0;i<StepStageCount;++i) {
        EXPECT_EQ(off.total[i].calls,0u);EXPECT_EQ(off.total[i].wall_ns,0u);
        const auto expected=i==static_cast<std::size_t>(StepStage::DiscardTrial)?0u:steps;
        EXPECT_EQ(on.total[i].calls,expected)<<StepStageNames[i];
        EXPECT_EQ(on.total[i].valid_samples,expected);EXPECT_EQ(on.total[i].failures,0u);
        EXPECT_EQ(on.last_step[i].calls,expected?1u:0u);
        if(i)stage_sum+=on.total[i].wall_ns;
    }
    EXPECT_LE(stage_sum,on.total[0].wall_ns);EXPECT_GT(on.total[0].wall_ns,0u);
}
} // namespace crash::cases::source_assembly_dynamics::test
