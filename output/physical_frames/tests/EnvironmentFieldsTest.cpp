#include "FieldsFixture.h"
#include "../EnvironmentFields.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::output::physical_frames::test {
namespace {
struct Combined {
    Fixture vehicle;
    std::vector<double> x{9,8,7,-0.,1,2,10,11,12,3,4,5,6,7,8,
        20,-1,-1,20,1,-1,20,1,1,20,-1,1},v=std::vector<double>(27,0.);
    std::vector<f::ShellBatchLayeredSection> q;
    std::uint8_t active[4]{1,0,1,1};
    detail::FixedEnvironmentField environment{3,{5,6,7,8},{{{20,-1,-1},{20,1,-1},{20,1,1},{20,-1,1}}}};
    Combined():q(vehicle.q,vehicle.q+3) {q.push_back(f::ShellBatchLayeredSection::GlobalLaw1());}
    void Stage() {
        detail::CheckFixedEnvironment(environment,x.data(),v.data(),9,q.data(),active,4);
        auto& out=vehicle.buffers.Staging();
        detail::StagePositions({3,1,4},x.data(),9,out);
        // Only after authentic full-family/suffix validation: unchanged strict
        // original vehicle-prefix staging, including zero-point role semantics.
        detail::StageLayered(vehicle.context,vehicle.mapping,QephFamily,q.data(),active,3,out,vehicle.buffers.flags);
        detail::StageLayered(vehicle.context,vehicle.mapping,T3Family,vehicle.t,vehicle.ta,1,out,vehicle.buffers.flags);
        detail::StageQbat(vehicle.context,vehicle.mapping,vehicle.b,vehicle.ba,1,out,vehicle.buffers.flags);
    }
};
}
TEST(EnvironmentCaptureValues, FullReadbackWallValidationPreservesOriginalVehicleFieldsAndZeroPoints) {
    Combined c;c.Stage();c.vehicle.buffers.Finish(c.vehicle.context,{});
    const auto& out=c.vehicle.buffers.frames[c.vehicle.buffers.selected];
    EXPECT_EQ(out.position_xyz,(std::vector<double>{3,4,5,-0.,1,2,6,7,8}));
    EXPECT_EQ(Bits(out.position_xyz[3]),Bits(-0.));EXPECT_EQ(out.plastic_points.size(),8u);
    EXPECT_EQ(c.vehicle.context.parents().size(),5u);EXPECT_EQ(c.q[3].law(),f::ShellSectionLaw::GlobalLaw1Npt0);
    EXPECT_EQ(c.q[3].plastic(),nullptr);EXPECT_EQ(c.q[3].elastic(),nullptr);
    // Legacy exact-count entry still rejects a silently appended parent.
    EXPECT_THROW(detail::StageLayered(c.vehicle.context,c.vehicle.mapping,QephFamily,c.q.data(),c.active,4,
        c.vehicle.buffers.Staging(),c.vehicle.buffers.flags),std::exception);
}
TEST(EnvironmentCaptureValues, WrongWallAndUnrenderedPhysicalFailuresKeepPublishedFrameThenRetry) {
    Combined c;c.Stage();c.vehicle.buffers.Finish(c.vehicle.context,{});
    const auto selected=c.vehicle.buffers.selected;const auto previous=c.vehicle.buffers.frames[selected];
    const auto original_x=c.x,original_v=c.v;const auto original_q=c.q;
    for(unsigned bad=0;bad<5;++bad) {
        if(bad==0)c.x[26]+=1;
        if(bad==1)c.v[15]=1;
        if(bad==2)c.active[3]=0;
        if(bad==3)c.q[3]=f::ShellBatchLayeredSection::RigidSkin();
        if(bad==4)c.x[6]=std::numeric_limits<double>::quiet_NaN(); // Not a render node.
        EXPECT_THROW(c.Stage(),std::exception);
        EXPECT_EQ(c.vehicle.buffers.selected,selected);
        EXPECT_EQ(c.vehicle.buffers.frames[selected].position_xyz,previous.position_xyz);
        c.x=original_x;c.v=original_v;c.q=original_q;c.active[3]=1;
    }
    EXPECT_NO_THROW(c.Stage());
}
TEST(EnvironmentCaptureValues, ExplicitSuffixBudgetCannotRelaxLegacyCountsOrHideFamilyRows) {
    Combined c;
    const auto f=detail::PlanBuffersWithEnvironment(c.vehicle.context,9,{4,1,1},{3,1,1},2048,{});
    EXPECT_EQ(f.physical_nodes,9u);EXPECT_EQ(f.layered_rows,4u);EXPECT_EQ(f.qbat_rows,1u);
    Limits cap;cap.host_bytes=f.peak_bytes;
    EXPECT_EQ(detail::PlanBuffersWithEnvironment(c.vehicle.context,9,{4,1,1},{3,1,1},2048,cap).peak_bytes,f.peak_bytes);
    --cap.host_bytes;
    EXPECT_THROW(detail::PlanBuffersWithEnvironment(c.vehicle.context,9,{4,1,1},{3,1,1},2048,cap),std::exception);
    EXPECT_THROW(detail::PlanBuffersWithEnvironment(c.vehicle.context,9,{5,1,1},{3,1,1},2048,{}),std::exception);
    EXPECT_THROW(detail::PlanBuffersWithEnvironment(c.vehicle.context,9,{4,2,1},{3,1,1},2048,{}),std::exception);
    EXPECT_THROW(detail::PlanBuffers(c.vehicle.context,9,4,1,1,2048,{}),std::exception);
}
}
