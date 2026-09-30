#pragma once
#include "Source.h"

namespace crash::modelio::vehicle::rigid_part::point_mass::detail {
// Internal typed-field parser only. Source::Prepare supplies authenticated,
// immutable RigidPartSource evidence; this helper creates no source authority.
std::vector<Record> Read(const std::vector<tied_shell::SourceEvidence>&,
                         double mass_to_kg,Limits);
}
