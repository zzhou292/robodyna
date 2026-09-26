#include "PartSets.h"
#include "modelio/tied_shell/Internal.h"
#include <algorithm>

namespace crash::modelio::self_contact::part_sets {
namespace {
bool Title(const std::string& keyword) {
    return keyword.size() >= 6 && keyword.substr(keyword.size() - 6) == "_TITLE";
}
}
std::vector<Set> Read(const std::vector<tied_shell::SourceEvidence>& candidates, std::size_t set_cap, std::size_t member_cap) {
    std::vector<Set> sets;
    tied_shell::Limits list_limits;
    list_limits.group_members = member_cap;
    for (std::size_t source_index = 0;
         source_index < candidates.size(); ++source_index) {
        const auto& source = candidates[source_index];
        const auto& keyword = source.block.keyword;
        if (keyword.rfind("*SET_PART_", 0) != 0) continue;
        const bool additive = keyword == "*SET_PART_ADD";
        const bool list = keyword == "*SET_PART_LIST" ||
            keyword == "*SET_PART_LIST_TITLE";
        output::Require(additive || list,
            "Unsupported original part-set operator in contact selection");
        const bool title = Title(keyword);
        const auto header = std::size_t(title);
        output::Require(source.cards.size() > header,
            "Original self-contact part set has no header");
        Set set;
        set.id = tied_shell::detail::CardId(
            source.cards[header].second, 0);
        set.source = source_index;
        set.additive = additive;
        set.members =
            tied_shell::detail::ListIds(source, title, list_limits);
        output::Require(!set.members.empty() && sets.size() < set_cap,
            "Original self-contact part set is empty or exceeds cap");
        sets.push_back(std::move(set));
    }
    std::sort(sets.begin(), sets.end(),
        [](const Set& a, const Set& b) { return a.id < b.id; });
    for (std::size_t i = 1; i < sets.size(); ++i)
        output::Require(sets[i - 1].id != sets[i].id,
            "Duplicate original self-contact part-set identity");
    return sets;
}

const Set& Find(const std::vector<Set>& sets, SourceId id) {
    const auto found = std::lower_bound(sets.begin(), sets.end(), id,
        [](const Set& set, SourceId value) { return set.id < value; });
    output::Require(found != sets.end() && found->id == id,
        "Original self-contact references a missing part set");
    return *found;
}

void Expand(const std::vector<Set>& sets, const Set& set,
    std::set<SourceId>& active, std::set<SourceId>& selected,
    std::vector<SourceId>& ordered, std::vector<std::size_t>& sources,
    std::size_t member_cap) {
    output::Require(active.insert(set.id).second,
        "Original self-contact part-set expansion contains a cycle");
    sources.push_back(set.source);
    if (set.additive) {
        for (const auto member : set.members)
            Expand(sets, Find(sets, member), active, selected,
                ordered, sources, member_cap);
    } else {
        for (const auto part : set.members) {
            output::Require(selected.insert(part).second &&
                    ordered.size() < member_cap,
                "Original self-contact selected part is duplicated or exceeds cap");
            ordered.push_back(part);
        }
    }
    active.erase(set.id);
}

} // namespace crash::modelio::self_contact::part_sets
