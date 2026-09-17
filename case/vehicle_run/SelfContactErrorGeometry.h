#pragma once

#include "output/ArtifactIO.h"

namespace tlfea::contact { struct SelfContactTransactionReport; }

namespace crash::cases::vehicle_run::detail {

// Diagnostic report values only; not a geometric replay or restart snapshot.
void AppendSelfContactErrorGeometry(
    output::Document&, const tlfea::contact::SelfContactTransactionReport&);

}  // namespace crash::cases::vehicle_run::detail
