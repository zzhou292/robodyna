#include "MaterialBridge.h"
#include <cfenv>
#include <cmath>
#include <cstring>
#include <stdexcept>
namespace crash::modelio::native_scene {
namespace {
void Require(bool valid,const char* message) {if(!valid)throw std::invalid_argument(message);}
std::uint64_t PositiveBits(double value) {
    // Both signed zeros represent the same zero hardening slope; the literal
    // source value itself remains in source_h_pa and no bit-identity is claimed.
    if(value==0)return 0;
    std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;
}
}
LinearHardeningBridge PrepareNativeHardening(double young,double poisson,double density,
    double yield,double source_h,tl::material::TabulatedShellPlasticityRate rate) {
    Require(std::fegetround()==FE_TONEAREST,"Native H bridge requires round-to-nearest arithmetic");
    Require(std::isfinite(young)&&young>0&&std::isfinite(source_h)&&source_h>=0&&source_h<=young,
        "Native H bridge requires finite E>0 and 0<=H<=E");
    // Deliberately use the agreed ratio association, not E*H/(E+H).
    // H<=E keeps the denominator in[1,2] and avoids a needless E*H product.
    const double ratio=source_h/young,denominator=1.+ratio;
    Require(std::isfinite(ratio)&&std::isfinite(denominator)&&denominator>=1.&&denominator<=2.,
        "Native H to ETAN denominator is invalid");
    const double etan=source_h/denominator;
    Require(std::isfinite(etan)&&etan>=0&&etan<young&&(source_h==0||etan>0),
        "Native H to ETAN conversion is underflowed or ill-conditioned");
    LinearHardeningBridge result;result.source_h_pa=source_h;result.derived_etan_pa=etan;
    Require(tl::material::PrepareLinearLaw44ShellPlasticity(young,poisson,density,{yield,etan},rate,
        result.parameters)==tl::material::TabulatedShellPlasticityStatus::Ok,
        "Derived native H bridge failed actual TL material preparation");
    result.prepared_h_pa=result.parameters.plastic_hardening_pa;
    Require(std::isfinite(result.prepared_h_pa)&&result.prepared_h_pa>=0&&
        (source_h==0||result.prepared_h_pa>0),"Prepared native H is invalid");
    const auto a=PositiveBits(source_h),b=PositiveBits(result.prepared_h_pa);
    result.prepared_h_ulp_difference=a>b?a-b:b-a;
    Require(result.prepared_h_ulp_difference<=2,"Prepared native H differs by more than2ULP");
    // A named, checked representation bridge is not source-parameter bit identity.
    result.exact_source_h_identity=false;return result;
}
} // namespace crash::modelio::native_scene
