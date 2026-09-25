#include "MaterialBridge.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace crash::modelio::native_scene {
namespace {
const tl::material::TabulatedShellPlasticityRate Rate{true,40,5,10000};
}
TEST(NativeSceneMaterial, NativeModulusIsAuthoritativeAndPreparedDifferenceIsVisible) {
    const auto bridge=PrepareNativeHardening(210000e6,.3,7850,250e6,1000e6,Rate);
    EXPECT_EQ(bridge.source_h_pa,1e9);EXPECT_NE(bridge.derived_etan_pa,bridge.source_h_pa);
    EXPECT_EQ(bridge.prepared_h_ulp_difference,1u);EXPECT_FALSE(bridge.exact_source_h_identity);
    EXPECT_EQ(bridge.prepared_h_pa,std::nextafter(1e9,std::numeric_limits<double>::infinity()));
    EXPECT_EQ(bridge.parameters.plastic_hardening_pa,bridge.prepared_h_pa);
    EXPECT_EQ(bridge.parameters.linear.tangent_modulus_pa,bridge.derived_etan_pa);
}
TEST(NativeSceneMaterial, GenericSupportedModuliHaveBoundedRoundTripAcrossScales) {
    unsigned admitted=0,rejected=0;
    for(double scale:{1.,1.125,1.75})for(int exponent=0;exponent<=49;exponent+=7)
      for(int ratio=0;ratio<=49;ratio+=7) {
        const double young=std::ldexp(scale,exponent),h=std::ldexp(young,-ratio);
        try {
          const auto bridge=PrepareNativeHardening(young,.25,7850,young/100,h,Rate);++admitted;
          EXPECT_LE(bridge.prepared_h_ulp_difference,2u);EXPECT_FALSE(bridge.exact_source_h_identity);
          const long double exact=(static_cast<long double>(young)*h)/(static_cast<long double>(young)+h);
          EXPECT_LE(std::abs(static_cast<long double>(bridge.derived_etan_pa)-exact),
              4*std::numeric_limits<double>::epsilon()*exact);
          EXPECT_NEAR(bridge.prepared_h_pa/h,1.,4*std::numeric_limits<double>::epsilon());
        } catch(const std::invalid_argument&) {++rejected;}
      }
    EXPECT_GT(admitted,150u);EXPECT_EQ(admitted+rejected,192u);
    const auto zero=PrepareNativeHardening(210e9,.3,7850,250e6,0,Rate);
    EXPECT_EQ(zero.derived_etan_pa,0);EXPECT_EQ(zero.prepared_h_pa,0);EXPECT_EQ(zero.prepared_h_ulp_difference,0u);
}
TEST(NativeSceneMaterial, InvalidOverflowAndUnderflowInputsDoNotYieldMaterial) {
    for(double h:{-1.,211e9,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
      EXPECT_THROW(PrepareNativeHardening(210e9,.3,7850,250e6,h,Rate),std::invalid_argument);
    EXPECT_THROW(PrepareNativeHardening(1e300,.3,7850,250e6,1e300,Rate),std::invalid_argument);
    EXPECT_THROW(PrepareNativeHardening(1e-300,.3,7850,1e-302,1e-300,Rate),std::invalid_argument);
    EXPECT_THROW(PrepareNativeHardening(210e9,.5,7850,250e6,1e9,Rate),std::invalid_argument);
    auto disabled=Rate;disabled.enabled=false;
    EXPECT_THROW(PrepareNativeHardening(210e9,.3,7850,250e6,1e9,disabled),std::invalid_argument);
}
}
