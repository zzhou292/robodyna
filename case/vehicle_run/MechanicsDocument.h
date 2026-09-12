#pragma once
#include "MechanicsTotals.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_run::detail {
output::Document MechanicsDocument(const MechanicsTotals&);
}
