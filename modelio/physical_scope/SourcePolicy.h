#pragma once
#include "PhysicalScope.h"
namespace crash::modelio::physical_scope {
// Both explicit support profiles retain the same complete structural sources.
// Their solid formulations remain distinguished by the solid source policy.
inline bool HasVehicleSupports(solid_source::Policy policy) noexcept {
    return policy == solid_source::Policy::OriginalVehicleSupportsV5 ||
           policy == solid_source::Policy::NativeConvertedSupportsV6;
}
} // namespace crash::modelio::physical_scope
