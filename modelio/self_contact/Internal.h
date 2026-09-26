#pragma once

#include "OriginalSelection.h"
#include "PartSets.h"
#include "modelio/tied_shell/Internal.h"

namespace crash::modelio::self_contact::detail {

using SourceSet = part_sets::Set; // Preserve the existing forecast type/size.

struct Draft {
    Data data;
    std::vector<tied_shell::SourceEvidence> candidates;
    std::vector<part_sets::Set> sets;
};

std::size_t Preflight(const source::CanonicalData&, std::size_t,
    std::size_t, Limits);
std::size_t OwnedPayload(const Data&, std::size_t);
void ReadSources(const source::CanonicalData&, const std::string&,
    const std::string&, Draft&, Limits);
void ResolveCards(Draft&, Limits);
void BuildCensus(const source::CanonicalData&, Draft&, Limits);

}  // namespace crash::modelio::self_contact::detail
