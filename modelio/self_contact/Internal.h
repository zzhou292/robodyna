#pragma once

#include "OriginalSelection.h"
#include "modelio/tied_shell/Internal.h"

namespace crash::modelio::self_contact::detail {

struct SourceSet {
    SourceId id = 0;
    std::size_t source = SIZE_MAX;
    bool additive = false;
    std::vector<SourceId> members;
};

struct Draft {
    Data data;
    std::vector<tied_shell::SourceEvidence> candidates;
    std::vector<SourceSet> sets;
};

std::size_t Preflight(const source::CanonicalData&, std::size_t,
    std::size_t, Limits);
std::size_t OwnedPayload(const Data&, std::size_t);
void ReadSources(const source::CanonicalData&, const std::string&,
    const std::string&, Draft&, Limits);
void ResolveCards(Draft&, Limits);
void BuildCensus(const source::CanonicalData&, Draft&, Limits);

}  // namespace crash::modelio::self_contact::detail
