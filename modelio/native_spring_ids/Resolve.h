#pragma once
#include "ImportContext.h"
#include "modelio/type13/SourceType13.h"
#include "modelio/type25/VehicleType25Source.h"
#include "modelio/type45/VehicleType45Source.h"
namespace crash::modelio::native_spring_ids {
// The profile reproduces the original import namespace for V5 participants.
// Namespace-only precursor IDs affect allocation, never physical K/M or nodes.
// Unready results contain no mapping rows; original and native IDs stay distinct.
Forecast Preflight(const type13::SourceType13&, const type25::VehicleType25Source&,
    const type45::VehicleType45Source&, const ImportContext&, Limits = {});
Resolution Resolve(const type13::SourceType13&, const type25::VehicleType25Source&,
    const type45::VehicleType45Source&, const ImportContext&, Limits = {});
} // namespace crash::modelio::native_spring_ids
