#include "NativePhysicalThickness.h"
#include "NativePointInternal.h"

namespace tl::qualification::law44 {
bool EvaluatePhysicalThickness(const Input& in,double layer,double thickness,
                              PhysicalThicknessResult& output) {
    if(!std::isfinite(layer)||layer<=0||!std::isfinite(thickness)||thickness<=0) return false;
    std::array<double,13> values{};
    if(!detail::EvaluateValues(in,true,layer,thickness,values)||values[8]<=0) return false;
    PhysicalThicknessResult candidate;
    if(!detail::DecodeResponse(in,values,candidate)) return false;
    candidate.reported_thickness_m=values[8];
    output=candidate; return true;
}
} // namespace tl::qualification::law44
