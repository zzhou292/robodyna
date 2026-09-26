#include "Internal.h"
#include "modelio/self_contact/PartSets.h"
#include "modelio/vehicle_source/SourceCards.h"

namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
namespace {
void OrdinaryHeader(const modelio::tied_shell::SourceEvidence& evidence) {
    const auto& keyword = evidence.block.keyword;
    const auto header = keyword == "*SET_PART_LIST_TITLE" ? std::size_t{1} : 0;
    if (evidence.cards.size() <= header) Reject(Status::UnsupportedSource, "Surface set has no ordinary source header");
    for (unsigned field = 1; field < 8; ++field) {
        const auto value = modelio::vehicle::detail::SourceScalar(evidence.cards[header].second, field);
        if (value && *value != 0) Reject(Status::UnsupportedSource, "Surface set has unsupported nonzero header controls");
    }
}
}
Controls ResolveSelectionControls(const modelio::self_contact::Data& selected, std::size_t cap) {
    using namespace modelio::self_contact;
    if (selected.profile != OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1 ||
        selected.source_fields.slave_set_type != 2 || selected.source_fields.master_set_id != 0 ||
        selected.sources.size() != 3 || selected.selected_part_ids.empty() || selected.selected_part_ids.size() > cap)
        Reject(Status::UnsupportedSource, "Initial surface requires one authenticated single-list PART chain");
    const auto sets = part_sets::Read(selected.sources, 3, cap);
    if (sets.size() != 2) Reject(Status::UnsupportedSource, "Multiple source surface clauses require native SET merge semantics");
    const auto& root = part_sets::Find(sets, selected.source_fields.slave_set_id);
    if (!root.additive || root.members.size() != 1 || !root.members[0])
        Reject(Status::UnsupportedSource, "Surface PART_ADD is not a single positive child");
    const auto& list = part_sets::Find(sets, root.members[0]);
    if (list.additive || list.members != selected.selected_part_ids || root.source == list.source)
        Reject(Status::UnsupportedSource, "Surface child is not the exact retained ordinary PART list");
    for (const auto* set : {&root, &list}) {
        if (set->source >= selected.sources.size()) Reject(Status::InvalidInput, "Set source ordinal is invalid");
        OrdinaryHeader(selected.sources[set->source]);
    }
    // convertsets: ordinary LIST -> one PART clause; single-positive-child ADD
    // -> one SET clause. CFG defaults and indexed unavailable fallback set
    // O/A/E to0; CREATE_SURFACE selects EXT1. Empty-destination SET insertion
    // copies the one child's surface; no multi-list union equivalence assumed.
    return {"a62_direct_fresh_single_list_PART_EXT1_OPT_O0_NADMESH0_NUMELTRIA0_v1"};
}
Controls ResolveControls(const Context& context, const Selection& selection, Limits limits) {
    const auto& physical = context.pre_correction().physical();
    if (&physical.shell_source().references().source().canonical().data() != &selection.canonical().data() ||
        context.provenance().source_digest != context.pre_correction().provenance().import_source_digest ||
        context.provenance().interfaces.disposition != nodal_correction::InterfaceDisposition::CompleteNoApplicableType24 ||
        !context.provenance().interfaces.checked_source_blocks)
        Reject(Status::UnsupportedSource, "Initial source lacks the shared closed fresh-import context");
    // The private corrected context was constructed through the supported
    // DirectKeywordR14FreshRadiossPoSortById import and complete generator
    // census. There is no native preload/adaptive source. At pinned a62 the
    // full converter inventory emits no /ADMESH; ordinary3D shell conversion
    // emits SHELL/SH3N, notTRIA. See ORIGINAL_PART_SURFACE_DEFAULTS evidence.
    return ResolveSelectionControls(selection.data(), limits.parts);
}
}
