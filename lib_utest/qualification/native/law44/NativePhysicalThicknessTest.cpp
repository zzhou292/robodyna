#include "NativePhysicalThickness.h"
#include "NativeRateTestSupport.h"
#include <cstring>
#include <limits>

namespace {
namespace native=tl::qualification::law44;
using namespace native::rate_test;
template<class T> bool Bits(const T& a,const T& b) {return std::memcmp(&a,&b,sizeof(T))==0;}

TEST(Law44PhysicalThickness, ActualNativeAdditionsCarryYieldedLoadingHoldAndUnloading) {
    const auto p=Prepare(); auto input=NativeInput(p);
    History actual_history; double actual_thickness=Thickness,native_thickness=Thickness;
    unsigned yielded=0,collapsed_rounding_differences=0; double yielded_before_hold=0;
    for(unsigned step=0;step<1728;++step) {
        SCOPED_TRACE(step);
        const double direction=step<960?1.:step<1088?0.:-.75;
        input.strain_increment={direction*2.e-5,direction*-7.e-6,direction*1.e-5,0.,0.};
        std::array<double,8> dx{};
        std::copy(input.strain_increment.begin(),input.strain_increment.end(),dx.begin());
        input.rate.total_shell_rate_per_s=native::NativeShellRate(dx,native_thickness,Dt);
        const double layer=.25*native_thickness;
        native::PhysicalThicknessResult expected;
        ASSERT_TRUE(native::EvaluatePhysicalThickness(input,layer,native_thickness,expected));
        native::Result unit;
        ASSERT_TRUE(native::Evaluate(input,unit));
        for(unsigned c=0;c<5;++c) EXPECT_DOUBLE_EQ(expected.stress[c],unit.stress[c]);
        EXPECT_DOUBLE_EQ(expected.plastic_strain,unit.plastic_strain);
        const double collapsed=native_thickness+unit.total_thickness_strain*layer;
        collapsed_rounding_differences+=!Bits(collapsed,expected.reported_thickness_m);

        material::TabulatedShellPlasticityInput update;
        std::copy(input.strain_increment.begin(),input.strain_increment.end(),update.strain_increment);
        update.transverse_shear_modulus=input.transverse_shear_modulus;
        update.dt=Dt;update.total_strain_rate_per_s=input.rate.total_shell_rate_per_s;
        PointResult actual;
        ASSERT_EQ(material::UpdateTabulatedShellPlasticity(p,actual_history,update,actual),Status::Ok);
        ComparePoint(actual,unit);
        const double actual_layer=.25*actual_thickness;
        actual_thickness+=actual.elastic_thickness_strain*actual_layer;
        actual_thickness+=actual.plastic_thickness_strain*actual_layer;
        Close(actual_thickness,expected.reported_thickness_m,2.e-14);
        if(expected.plastic_increment>0) ++yielded;
        if(step==959) yielded_before_hold=expected.plastic_strain;
        // Source VP2 rate filtering decays during zero-strain hold. The lowered
        // native yield can continue plastic flow; a rate-independent elastic
        // hold assertion would contradict the selected source law.
        if(step>=960&&step<1088) EXPECT_GE(expected.plastic_strain,yielded_before_hold);
        actual_history=actual.history;
        input.accepted_stress=expected.stress;
        input.accepted_plastic_strain=expected.plastic_strain;
        input.rate.accepted_filtered_rate_per_s=expected.filtered_rate_per_s;
        native_thickness=expected.reported_thickness_m;
    }
    EXPECT_GT(yielded,100u); EXPECT_GT(yielded_before_hold,.001);
    EXPECT_GT(std::abs(native_thickness-Thickness),1.e-7);
    EXPECT_GT(collapsed_rounding_differences,0u);
    RecordProperty("combined_increment_rounding_differences",collapsed_rounding_differences);
}

TEST(Law44PhysicalThickness, InvalidAndLateVanishedThicknessPreserveOutputAndRetry) {
    const auto p=Prepare(false);auto input=NativeInput(p);
    input.strain_increment={1.e-5,0.,2.e-5,0.,0.};
    native::PhysicalThicknessResult output;
    ASSERT_TRUE(native::EvaluatePhysicalThickness(input,.25*Thickness,Thickness,output));
    const auto saved=output;
    EXPECT_FALSE(native::EvaluatePhysicalThickness(input,-1.,Thickness,output));
    EXPECT_TRUE(Bits(output,saved));
    auto bad=input;bad.accepted_stress[4]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(native::EvaluatePhysicalThickness(bad,.25*Thickness,Thickness,output));
    EXPECT_TRUE(Bits(output,saved));
    // Admitted finite constitutive input; the physical layer contribution is
    // large enough to make the running thickness negative in native SIGEPS44C.
    EXPECT_FALSE(native::EvaluatePhysicalThickness(input,1.,1.e-12,output));
    EXPECT_TRUE(Bits(output,saved));
    ASSERT_TRUE(native::EvaluatePhysicalThickness(input,.25*Thickness,Thickness,output));
    EXPECT_TRUE(Bits(output,saved));
}
} // namespace
