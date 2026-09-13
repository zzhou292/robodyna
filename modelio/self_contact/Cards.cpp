#include "Internal.h"

#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include "modelio/vehicle_source/SourceCards.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace crash::modelio::self_contact::detail {
namespace {

bool Title(const std::string& keyword) {
    return keyword.size() >= 6 &&
        keyword.substr(keyword.size() - 6) == "_TITLE";
}

bool Blank(const std::string& card) {
    return !modelio::assembly::reader::auxiliary::Trim(card).size();
}

std::optional<double> Field(const tied_shell::SourceEvidence& source,
    std::size_t card, unsigned field) {
    output::Require(card < source.cards.size(),
        "Original self-contact card row is missing");
    return vehicle::detail::SourceScalar(source.cards[card].second, field);
}

void ZeroOrBlank(const tied_shell::SourceEvidence& source,
    std::size_t card, unsigned begin, unsigned end) {
    for (unsigned field = begin; field < end; ++field) {
        const auto value = Field(source, card, field);
        output::Require(!value || *value == 0,
            "Original self-contact unsupported source field is nonzero");
    }
}

std::size_t ContactSource(const Draft& draft) {
    std::size_t found = SIZE_MAX;
    for (std::size_t i = 0; i < draft.candidates.size(); ++i) {
        if (draft.candidates[i].block.keyword !=
            "*CONTACT_AUTOMATIC_SINGLE_SURFACE")
            continue;
        output::Require(found == SIZE_MAX,
            "Duplicate original automatic-single-surface source");
        found = i;
    }
    output::Require(found != SIZE_MAX,
        "Missing original automatic-single-surface source");
    return found;
}

void ParseContact(const tied_shell::SourceEvidence& source,
    OriginalContactFields& output) {
    output::Require(source.cards.size() == 8 &&
            !Blank(source.cards[0].second) &&
            !Blank(source.cards[1].second) &&
            Blank(source.cards[2].second) &&
            !Blank(source.cards[3].second) &&
            Blank(source.cards[4].second) &&
            !Blank(source.cards[5].second) &&
            Blank(source.cards[6].second) &&
            Blank(source.cards[7].second),
        "Original automatic-single-surface card shape differs");
    const auto ssid = Field(source, 0, 0);
    const auto msid = Field(source, 0, 1);
    const auto sstyp = Field(source, 0, 2);
    output::Require(ssid && *ssid > 0 && std::floor(*ssid) == *ssid &&
            *ssid <= UINT32_MAX && msid && *msid == 0 &&
            sstyp && *sstyp == 2,
        "Original automatic-single-surface set identity/type differs");
    ZeroOrBlank(source, 0, 3, 8);
    output.slave_set_id = static_cast<SourceId>(*ssid);
    output.master_set_id = 0;
    output.slave_set_type = 2;

    output.static_friction = Field(source, 1, 0);
    output.dynamic_friction = Field(source, 1, 1);
    output.decay_coefficient = Field(source, 1, 2);
    output::Require(output.static_friction &&
            output.dynamic_friction && output.decay_coefficient &&
            *output.static_friction >= 0 &&
            *output.dynamic_friction >= 0 &&
            *output.decay_coefficient >= 0,
        "Original automatic-single-surface friction/decay fields differ");
    ZeroOrBlank(source, 1, 3, 8);

    output.soft = Field(source, 3, 0);
    output::Require(output.soft && *output.soft == 1,
        "Original automatic-single-surface SOFT field differs");
    ZeroOrBlank(source, 3, 1, 8);

    const auto igap = Field(source, 5, 0);
    output.ignore_initial_penetration = Field(source, 5, 1);
    output::Require(!igap &&
            output.ignore_initial_penetration &&
            *output.ignore_initial_penetration == 1,
        "Original automatic-single-surface IGAP/IGNORE fields differ");
    ZeroOrBlank(source, 5, 2, 8);
}

std::vector<SourceSet> Sets(const Draft& draft, Limits limits) {
    std::vector<SourceSet> sets;
    tied_shell::Limits list_limits;
    list_limits.group_members = limits.parts;
    for (std::size_t source_index = 0;
         source_index < draft.candidates.size(); ++source_index) {
        const auto& source = draft.candidates[source_index];
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
        SourceSet set;
        set.id = tied_shell::detail::CardId(
            source.cards[header].second, 0);
        set.source = source_index;
        set.additive = additive;
        set.members =
            tied_shell::detail::ListIds(source, title, list_limits);
        output::Require(!set.members.empty() && sets.size() < limits.blocks,
            "Original self-contact part set is empty or exceeds cap");
        sets.push_back(std::move(set));
    }
    std::sort(sets.begin(), sets.end(),
        [](const SourceSet& a, const SourceSet& b) { return a.id < b.id; });
    for (std::size_t i = 1; i < sets.size(); ++i)
        output::Require(sets[i - 1].id != sets[i].id,
            "Duplicate original self-contact part-set identity");
    return sets;
}

const SourceSet& Find(const std::vector<SourceSet>& sets, SourceId id) {
    const auto found = std::lower_bound(sets.begin(), sets.end(), id,
        [](const SourceSet& set, SourceId value) { return set.id < value; });
    output::Require(found != sets.end() && found->id == id,
        "Original self-contact references a missing part set");
    return *found;
}

void Expand(const std::vector<SourceSet>& sets, const SourceSet& set,
    std::set<SourceId>& active, std::set<SourceId>& selected,
    std::vector<SourceId>& ordered, std::vector<std::size_t>& sources,
    Limits limits) {
    output::Require(active.insert(set.id).second,
        "Original self-contact part-set expansion contains a cycle");
    sources.push_back(set.source);
    if (set.additive) {
        for (const auto member : set.members)
            Expand(sets, Find(sets, member), active, selected,
                ordered, sources, limits);
    } else {
        for (const auto part : set.members) {
            output::Require(selected.insert(part).second &&
                    ordered.size() < limits.parts,
                "Original self-contact selected part is duplicated or exceeds cap");
            ordered.push_back(part);
        }
    }
    active.erase(set.id);
}

}  // namespace

void ResolveCards(Draft& draft, Limits limits) {
    const auto contact_index = ContactSource(draft);
    const auto& contact = draft.candidates[contact_index];
    ParseContact(contact, draft.data.source_fields);
    draft.sets = Sets(draft, limits);
    const auto& root = Find(
        draft.sets, draft.data.source_fields.slave_set_id);
    output::Require(root.additive,
        "Original automatic-single-surface root is not SET_PART_ADD");

    std::set<SourceId> active, selected;
    std::vector<std::size_t> set_sources;
    Expand(draft.sets, root, active, selected,
        draft.data.selected_part_ids, set_sources, limits);
    output::Require(!draft.data.selected_part_ids.empty(),
        "Original self-contact selection resolved no parts");

    draft.data.sources.reserve(1 + set_sources.size());
    draft.data.sources.push_back(contact);
    std::set<std::size_t> retained;
    for (const auto source : set_sources) {
        if (retained.insert(source).second)
            draft.data.sources.push_back(draft.candidates[source]);
    }
}

}  // namespace crash::modelio::self_contact::detail
