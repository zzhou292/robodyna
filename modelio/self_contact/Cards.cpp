#include "Internal.h"
#include "PartSets.h"

#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include "modelio/vehicle_source/SourceCards.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace crash::modelio::self_contact::detail {
namespace {

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


}  // namespace

void ResolveCards(Draft& draft, Limits limits) {
    const auto contact_index = ContactSource(draft);
    const auto& contact = draft.candidates[contact_index];
    ParseContact(contact, draft.data.source_fields);
    draft.sets = part_sets::Read(draft.candidates, limits.blocks, limits.parts);
    const auto& root = part_sets::Find(
        draft.sets, draft.data.source_fields.slave_set_id);
    output::Require(root.additive,
        "Original automatic-single-surface root is not SET_PART_ADD");

    std::set<SourceId> active, selected;
    std::vector<std::size_t> set_sources;
    part_sets::Expand(draft.sets, root, active, selected,
        draft.data.selected_part_ids, set_sources, limits.parts);
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
