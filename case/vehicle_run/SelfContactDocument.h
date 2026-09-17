#pragma once

#include "SelfContactTotals.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_self_contact { class SelfContactStageError; }

namespace crash::cases::vehicle_run::detail {

output::Document SelfContactDocument(const SelfContactTotals&);
output::Document SelfContactErrorDocument(
    const vehicle_self_contact::SelfContactStageError&);

}  // namespace crash::cases::vehicle_run::detail
