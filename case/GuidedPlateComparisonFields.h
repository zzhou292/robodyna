#pragma once
#include "GuidedPlateStudy.h"
#include "output/ArtifactIO.h"

namespace crash::case_data::comparison_io {
// Shared numerical fields; callers own the distinct refinement/wall schemas
// and authenticated input identities. No comparison math is implemented here.
inline void Append(output::Document& d,const GuidedStudyComparison& c) {
    namespace io=output;
    io::Boolean(d,"passed",c.passed);io::String(d,"diagnostic",c.diagnostic);
    io::Number(d,"displacement_ratio",c.displacement_ratio);io::Number(d,"velocity_ratio",c.velocity_ratio);
    io::Number(d,"rotation_ratio",c.rotation_ratio);io::Number(d,"force_ratio",c.force_ratio);
    io::Number(d,"impulse_ratio",c.impulse_ratio);io::Number(d,"energy_ratio",c.energy_ratio);
    io::Number(d,"event_ratio",c.event_ratio);io::Number(d,"penetration_ratio",c.penetration_ratio);
    io::Boolean(d,"energy_envelopes",c.energy_envelopes);io::Boolean(d,"deforming_contact_evidence",c.deforming_contact_evidence);
    io::Boolean(d,"events_complete",c.events_complete);
}
} // namespace crash::case_data::comparison_io
