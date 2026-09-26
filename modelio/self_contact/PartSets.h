#pragma once
#include "modelio/tied_shell/TiedShellDeclaration.h"
#include <set>

namespace crash::modelio::self_contact::part_sets {
using SourceId = assembly::SourceId;
struct Set {
    SourceId id = 0;
    std::size_t source = SIZE_MAX;
    bool additive = false;
    std::vector<SourceId> members;
};
std::vector<Set> Read(const std::vector<tied_shell::SourceEvidence>&,
    std::size_t set_cap, std::size_t member_cap);
const Set& Find(const std::vector<Set>&, SourceId);
// Exact source traversal order. Duplicate selected IDs and cycles reject.
void Expand(const std::vector<Set>&, const Set&, std::set<SourceId>& active,
    std::set<SourceId>& selected, std::vector<SourceId>& ordered,
    std::vector<std::size_t>& source_indices, std::size_t member_cap);
} // namespace crash::modelio::self_contact::part_sets
