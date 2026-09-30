#pragma once
#include "Values.h"
#include "output/ArtifactIO.h"
#include <iosfwd>
namespace crash::cases::vehicle_run::contact_diagnostics {
output::Document Document(const Snapshot&);
void WriteProgress(std::ostream&,const Snapshot&);
} // namespace crash::cases::vehicle_run::contact_diagnostics
