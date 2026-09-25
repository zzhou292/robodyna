#include "chrono/ReplayClipping.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
namespace crash::visual::test {
namespace {
ReplayCamera Camera(double scale=1) {
    ReplayCamera c;c.position={.04*scale,-.065*scale,.045*scale};c.target={0,0,.0005*scale};return c;
}
ReplayBounds Bounds(double scale=1) {return {{-.02*scale,-.02*scale,-.003*scale},{.02*scale,.02*scale,.003*scale}};}
void Encloses(const ReplayCamera& c,const ReplayBounds& b,const ReplayClipping& clip) {
    ASSERT_GT(clip.minimum_depth_m,0);
    ASSERT_LT(clip.near_m,clip.minimum_depth_m);ASSERT_GT(clip.far_m,clip.maximum_depth_m);
    for(unsigned i=0;i<8;++i) {
        double z=0;
        for(unsigned a=0;a<3;++a) z+=((i&(1u<<a)?b.high[a]:b.low[a])-c.position[a])*
            ((c.target[a]-c.position[a])/clip.camera_distance_m);
        EXPECT_GT(z,clip.near_m);EXPECT_LT(z,clip.far_m);
        // Vulkan reverse-depth projection leaves every corner inside [0,1].
        const double depth=-clip.near_m/(clip.far_m-clip.near_m)+
            (clip.far_m*clip.near_m)/(z*(clip.far_m-clip.near_m));
        EXPECT_GT(depth,0);EXPECT_LT(depth,1);
    }
}
}
TEST(ReplayClipping, CentimetreAndVehicleScenesKeepEveryDepthWithoutChangingPoseOrGeometry) {
    ReplayClipping small;
    for(double scale:{1.,.01,100.,1000.}) {
        const auto c=Camera(scale);const auto b=Bounds(scale);const auto before_c=c;const auto before_b=b;
        ReplayClipping clip;ASSERT_TRUE(MakeReplayClipping(c,b,clip));Encloses(c,b,clip);
        EXPECT_EQ(c.position,before_c.position);EXPECT_EQ(c.target,before_c.target);
        EXPECT_EQ(b.low,before_b.low);EXPECT_EQ(b.high,before_b.high);
        if(scale==1) {small=clip;EXPECT_LT(clip.near_m,.0001);}
        else {EXPECT_NEAR(clip.near_m,small.near_m*scale,clip.near_m*1e-14);
            EXPECT_NEAR(clip.far_m,small.far_m*scale,clip.far_m*1e-14);}
    }
}
TEST(ReplayClipping, WholeMotionBoundsAndRotatedLookDirectionSetConservativeFarPlane) {
    auto c=Camera();auto b=Bounds();b.low[0]=-.04;b.high[2]=.04;
    ReplayClipping clip;ASSERT_TRUE(MakeReplayClipping(c,b,clip));Encloses(c,b,clip);
    c.position={0,0,.05};c.target={0,0,0};b={{-.001,-.001,.049},{.001,.001,.0499}};
    ASSERT_TRUE(MakeReplayClipping(c,b,clip));Encloses(c,b,clip);
    EXPECT_LE(clip.near_m,.5*clip.minimum_depth_m);
}
TEST(ReplayClipping, EyePlaneCrossingIsReportedWithoutInventingVisibilityBehindCamera) {
    auto c=Camera();c.position={0,0,0};c.target={0,0,1};
    ReplayClipping clip;ASSERT_TRUE(MakeReplayClipping(c,{{-1,-1,-1},{1,1,1}},clip));
    EXPECT_LT(clip.minimum_depth_m,0);EXPECT_GT(clip.maximum_depth_m,0);
    EXPECT_GT(clip.near_m,0);EXPECT_GT(clip.far_m,1);
    EXPECT_FALSE(MakeReplayClipping(c,{{-1,-1,-2},{1,1,-1}},clip));
}
TEST(ReplayClipping, MalformedAndUnrepresentableInputsPreserveAllDiagnosticsAndAllowRetry) {
    ReplayClipping out;ASSERT_TRUE(MakeReplayClipping(Camera(),Bounds(),out));const auto before=out;
    for(unsigned fault=0;fault<8;++fault) {
        auto c=Camera();auto b=Bounds();
        switch(fault) {
            case 0:c.target=c.position;break;
            case 1:c.position[0]=std::numeric_limits<double>::infinity();break;
            case 2:b.low[0]=std::numeric_limits<double>::quiet_NaN();break;
            case 3:b.low[0]=b.high[0]+1;break;
            case 4:b.low=b.high;break;
            case 5:c.position[0]=1e308;c.target[0]=-1e308;break;
            case 6:c=Camera(1e-100);b=Bounds(1e-100);break;
            case 7:c=Camera(1e100);b=Bounds(1e100);break;
        }
        EXPECT_FALSE(MakeReplayClipping(c,b,out));EXPECT_EQ(std::memcmp(&out,&before,sizeof(out)),0);
    }
    ASSERT_TRUE(MakeReplayClipping(Camera(),Bounds(),out));EXPECT_EQ(std::memcmp(&out,&before,sizeof(out)),0);
}
} // namespace crash::visual::test
