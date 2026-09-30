#pragma once
#include "VehicleConnectivity.h"
#include "Components.h"
#include "Relations.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
using output::Require;
Counts Extents(const VehiclePhysicalAttachments&, const joints::VehicleJointModel* = nullptr);
Forecast Budget(const VehiclePhysicalAttachments&, const Counts&, Limits, std::size_t fixed,
                const joints::VehicleJointModel* = nullptr);
void VisitShells(const VehiclePhysicalAttachments&, Relations&);
void VisitElements(const VehiclePhysicalAttachments&, Relations&);
void VisitConstraints(const VehiclePhysicalAttachments&, Relations&);
void CheckJoints(const VehiclePhysicalAttachments&, const joints::VehicleJointModel&);
void VisitJoints(const joints::VehicleJointModel&, Relations&);
} // namespace crash::cases::vehicle_startup::connectivity::detail
