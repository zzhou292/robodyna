#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace crash::cases::source_assembly_dynamics {
inline constexpr const char* NativeMinimumGuardFailure="Native area/thickness/timestep diagnostic guard failed";
struct NativeGuardValues {double area_ratio=0,thickness_ratio=0,native_dt=0;};
struct NativeGuardParent {std::uint64_t source=0;NativeGuardValues values;};
struct NativeGuardLimits {double area_ratio=0,thickness_ratio=0,requested_dt=0,native_dt_fraction=0;};
struct NativeGuardDetail {std::uint64_t source=0;const char* message=NativeMinimumGuardFailure;double measured=0,limit=0;};
// Diagnostics only: call AFTER the existing aggregate guard has rejected.
// Preserve its area -> thickness -> invalid DTEL -> step-size priority, then
// report the first failing parent in that family's existing source order.
// A contradictory aggregate without a matching parent keeps source=0.
template<class ParentAt> NativeGuardDetail AttributeNativeGuard(const NativeGuardValues& aggregate,
    const NativeGuardLimits& limits,std::size_t count,ParentAt parent_at) {
    enum class Kind {Area,Thickness,InvalidDt,Step};Kind kind;
    if(!std::isfinite(aggregate.area_ratio)||aggregate.area_ratio<limits.area_ratio)kind=Kind::Area;
    else if(!std::isfinite(aggregate.thickness_ratio)||aggregate.thickness_ratio<limits.thickness_ratio)kind=Kind::Thickness;
    else if(!std::isfinite(aggregate.native_dt)||!(aggregate.native_dt>0))kind=Kind::InvalidDt;
    else kind=Kind::Step;
    const auto failed=[&](const NativeGuardValues& v) {
        if(kind==Kind::Area)return !std::isfinite(v.area_ratio)||v.area_ratio<limits.area_ratio;
        if(kind==Kind::Thickness)return !std::isfinite(v.thickness_ratio)||v.thickness_ratio<limits.thickness_ratio;
        if(kind==Kind::InvalidDt)return !std::isfinite(v.native_dt)||!(v.native_dt>0);
        return limits.requested_dt>v.native_dt*limits.native_dt_fraction;
    };
    NativeGuardParent selected{0,aggregate};
    for(std::size_t i=0;i<count;++i) {const auto p=parent_at(i);if(failed(p.values)) {selected=p;break;}}
    const auto& v=selected.values;
    if(kind==Kind::Area)return {selected.source,"Native parent minimum area ratio guard failed",v.area_ratio,limits.area_ratio};
    if(kind==Kind::Thickness)return {selected.source,"Native parent minimum thickness ratio guard failed",v.thickness_ratio,limits.thickness_ratio};
    if(kind==Kind::InvalidDt)return {selected.source,"Native parent DTEL must be finite and positive",v.native_dt,0};
    return {selected.source,"Requested step exceeds native parent DTEL fraction",limits.requested_dt,v.native_dt*limits.native_dt_fraction};
}
}
