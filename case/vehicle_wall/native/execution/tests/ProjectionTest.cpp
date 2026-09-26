#include "case/vehicle_runtime/Packing.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::vehicle_wall::native::execution_test {
namespace rt=vehicle_runtime;
namespace fe=tl::fea;
namespace {
rt::detail::OwnerPacking Input() {
    rt::detail::OwnerPacking p;
    p.position.resize(9);p.velocity={2.,-0.,3.,2.,-0.,3.,2.,-0.,3.};
    p.spin.resize(9);p.orientation={1,0,0,0,1,0,0,0,1,0,0,0};
    p.mass={4,5,6};p.inertia={2,3,0};p.inverse_mass={.25,.2,1./6};p.inverse_inertia={.5,1./3,0};
    p.fixed={0,0,0};p.rotation_fixed={0,0,0};p.rotation_present={1,1,0};return p;
}
fe::ShellBatchStartup Startup(){return {fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,{2.,-0.,3.}};}
std::vector<std::uint64_t> Values(const rt::detail::OwnerPacking& p) {
    std::vector<std::uint64_t> out;
    for(const auto* row:{&p.position,&p.velocity,&p.spin,&p.orientation,&p.mass,&p.inertia,&p.inverse_mass,&p.inverse_inertia})
        for(double x:*row)out.push_back(output::Bits(x));
    for(const auto* row:{&p.fixed,&p.rotation_fixed,&p.rotation_present})for(auto x:*row)out.push_back(x);
    return out;
}
}
TEST(EnvelopeProjectionValues, FixedWallHasZeroVelocityAndInverseButRealRawMassInertia) {
    auto p=Input();const auto before=p;
    const std::uint8_t fixed[]{0,7,0},rotation[]{0,1,0};
    rt::detail::ApplyConstrainedStartup(p,Startup(),{fixed,3},{rotation,3});
    EXPECT_EQ(p.mass,before.mass);EXPECT_EQ(p.inertia,before.inertia);
    EXPECT_EQ(p.position,before.position);EXPECT_EQ(p.orientation,before.orientation);EXPECT_EQ(p.spin,before.spin);
    EXPECT_EQ(p.inverse_mass[1],0);EXPECT_EQ(p.inverse_inertia[1],0);
    for(unsigned k=0;k<3;++k)EXPECT_EQ(output::Bits(p.velocity[3+k]),output::Bits(0.));
    EXPECT_EQ(output::Bits(p.velocity[1]),output::Bits(-0.));
    EXPECT_EQ(output::Bits(p.velocity[7]),output::Bits(-0.));
    EXPECT_EQ(p.inverse_mass[0],before.inverse_mass[0]);EXPECT_EQ(p.rotation_present,before.rotation_present);
}
TEST(EnvelopeProjectionValues, ExistingMasksAreRetainedByOrWithoutFixingOtherDirections) {
    auto p=Input();p.fixed[0]=1;p.velocity[0]=0;
    const std::uint8_t added[]{2,0,0},rotation[]{0,0,0};
    rt::detail::ApplyConstrainedStartup(p,Startup(),{added,3},{rotation,3});
    EXPECT_EQ(p.fixed[0],3);EXPECT_EQ(p.velocity[0],0);EXPECT_EQ(p.velocity[1],0);EXPECT_EQ(p.velocity[2],3);
    EXPECT_EQ(p.inverse_mass[0],.25);EXPECT_EQ(p.inverse_inertia[0],.5);
}
TEST(EnvelopeProjectionValues, LateInvalidMaskAliasDescriptorAndReciprocalPreserveEveryLane) {
    for(unsigned variant=0;variant<6;++variant) {
        auto p=Input();auto startup=Startup();std::uint8_t fixed[]{0,7,0},rotation[]{0,1,0};
        tl::util::ConstView<std::uint8_t> fm{fixed,3},rm{rotation,3};
        if(variant==0)fixed[2]=8;
        if(variant==1)rotation[2]=1;
        if(variant==2)startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;
        if(variant==3)p.velocity[8]=4;
        if(variant==4)fm={p.fixed.data(),3};
        if(variant==5)p.inverse_mass[2]=std::numeric_limits<double>::infinity();
        const auto before=Values(p);
        EXPECT_THROW(rt::detail::ApplyConstrainedStartup(p,startup,fm,rm),std::exception);
        EXPECT_EQ(Values(p),before);
    }
    auto p=Input();const std::uint8_t fixed[]{0,7,0},rotation[]{0,1,0};
    EXPECT_NO_THROW(rt::detail::ApplyConstrainedStartup(p,Startup(),{fixed,3},{rotation,3}));
}
}
