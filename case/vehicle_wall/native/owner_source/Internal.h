#pragma once
#include "../EnvelopeOwnerSource.h"
#include "case/vehicle_startup/joints/SourceMapping.h"
#include "modelio/type45/SourcePolicy.h"
#include "lib_utils/BoundedArena.h"
namespace crash::cases::vehicle_wall::native::owner_source_detail {
using output::Require;
void Check(const EnvelopeExecutionSource&,const vehicle_startup::TiedSearchPostKinChk&,
    const modelio::type45::VehicleType45Source&,EnvelopeOwnerLimits);
}
