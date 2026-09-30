#pragma once
#include "ActualSupport.h"
namespace crash::modelio::physical_scope::test {
struct VehicleSupportInputs {
    OriginalInputs sources;
    beam18::Source structural;
    PhysicalScope scope;
    explicit VehicleSupportInputs(solid_source::Policy policy)
        : sources(policy),
          structural(beam18::Source::Prepare(vehicle::test::Canonical(),sources.member,
              beam18::Policy::OriginalCircularFourPointLaw44V1)),
          scope(PhysicalScope::PrepareVehicleSupports(sources.masses,sources.tied,sources.beams,
              sources.solids,structural)) {}
};
inline const VehicleSupportInputs& V5Supports(){static const VehicleSupportInputs x(solid_source::Policy::OriginalVehicleSupportsV5);return x;}
inline const VehicleSupportInputs& V6Supports(){static const VehicleSupportInputs x(solid_source::Policy::NativeConvertedSupportsV6);return x;}
} // namespace crash::modelio::physical_scope::test
