#include "Internal.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <climits>
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
bool OrdinaryNativeProperty(const modelio::assembly::Material& material, const modelio::assembly::Section& section) noexcept {
    const auto& keyword=material.source.keyword;
    const bool ordinary=keyword=="*MAT_ELASTIC" || keyword=="*MAT_RIGID" || keyword=="*MAT_020" ||
        keyword=="*MAT_PIECEWISE_LINEAR_PLASTICITY" || keyword=="*MAT_024" ||
        keyword=="*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY" || keyword=="*MAT_123";
    // convertprops p_ConvertSectionShell: a valid IRID selects another branch
    // before the ordinary TYPE1 arm. Reuse parsed authentic fields, no parser.
    return ordinary && section.source.keyword=="*SECTION_SHELL" && section.cards.size()>=2 &&
        section.cards[0].values.size()==8 &&
        (!section.cards[0].values[5] || *section.cards[0].values[5]==0.);
}
void Check(const Mixed& mixed, const GapOperands& gaps, Limits limits) {
    const Limits hard;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes && limits.nodes && limits.nodes <= hard.nodes &&
        limits.primaries && limits.primaries <= hard.primaries && limits.mains && limits.mains <= hard.mains &&
        limits.parts && limits.parts <= hard.parts && limits.metadata_bytes && limits.metadata_bytes <= hard.metadata_bytes,
        "Invalid bounded post-GAPM source limits");
    const auto& initial = mixed.initial();
    const auto& geometry = initial.geometry();
    const auto& context = initial.context();
    const auto& other = gaps.corrected();
    const auto& side = mixed.sides();
    Require(context.coefficients().data() == other.coefficients().data() &&
        context.coefficients().size() == other.coefficients().size() &&
        &context.pre_correction().physical().shell_source().references().source().canonical().data() ==
            &other.pre_correction().physical().shell_source().references().source().canonical().data(),
        "Post-GAPM and gap operands must retain the exact same corrected physical source");
    Require(gaps.provenance().source_digest == context.provenance().source_digest &&
        mixed.provenance().source_digest == context.provenance().source_digest &&
        gaps.counts().nodes == geometry.nodes.size() && gaps.counts().shells == geometry.shells.size() &&
        gaps.proof().no_retained_trusses && gaps.proof().checked_keywords &&
        gaps.proof().order == gap_operands::Order::CertifiedFiniteGapMaxima,
        "Post-GAPM requires the complete qualified physical gap operands");
    Require(initial.certificate().consumed_order_complete && initial.certificate().membership_complete &&
        mixed.certificate().complete_origins && mixed.certificate().complete_solid_flags &&
        mixed.certificate().unique_selected_membership && side.source_generation &&
        side.node_count == geometry.nodes.size() && side.primary_count &&
        side.shell_primary_count <= side.primary_count && side.main_count == side.primary_count+side.shell_primary_count,
        "Post-GAPM requires authentic complete mixed source sides and origin maps");
    const auto& selected = initial.selection().data();
    Require(selected.profile == modelio::self_contact::OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1 &&
        selected.source_fields.slave_set_type == 2 && selected.source_fields.master_set_id == 0 &&
        selected.sources.size() == 3 && initial.provenance().control_rule ==
            "a62_direct_fresh_single_list_PART_EXT1_OPT_O0_NADMESH0_NUMELTRIA0_v1",
        "Post-GAPM roster/default controls require the authenticated single S1 PART clause");
    if (context.part_controls().size() > limits.parts)
        Reject(Status::ResourceLimit, "Post-GAPM complete part controls exceed capacity");
    if (geometry.nodes.size() > limits.nodes || side.primary_count > limits.primaries || side.main_count > limits.mains)
        Reject(Status::ResourceLimit, "Post-GAPM source extents exceed capacity");
    Require(tlfea::contact::self_contact_filters::CompatibleHostArithmetic(),
        "Post-GAPM requires native round-to-nearest and gradual underflow");
    std::uint64_t previous = 0;
    for (const auto& node : geometry.nodes) {
        Require(node.source_id > previous && node.source_id <= INT_MAX, "Post-GAPM native ITAB source identities differ");
        previous = node.source_id;
    }
}
void Maps(const Mixed& source, Values& values) {
    const auto& input = source.initial().geometry();
    values.node_ids.reserve(input.nodes.size());
    values.positions.reserve(3*input.nodes.size());
    for (const auto& node : input.nodes) {
        values.node_ids.push_back(node.source_id);
        values.positions.push_back(node.native_position.x);
        values.positions.push_back(node.native_position.y);
        values.positions.push_back(node.native_position.z);
    }
    SourceNodeRosters(input, source.initial().faces(), values.secondary_nodes, values.main_nodes);
}
void SourceNodeRosters(const coated::Inputs& input, const std::vector<initial_surfaces::Face>& faces,
    std::vector<std::uint32_t>& secondary_output, std::vector<std::uint32_t>& main_output) {
    Require(!input.nodes.empty() && input.nodes.size()<=524288 && faces.size()<=1048576,
        "Source node roster extent exceeds bounded domain");
    std::uint64_t previous=0;
    for(const auto& node:input.nodes) {
        Require(node.source_id>previous && node.source_id<=INT_MAX, "Source node roster ITAB domain differs");
        previous=node.source_id;
    }
    std::vector<std::uint32_t> secondary, main;
    // I25SURFI S1 loop: first encounter in the actual consumed CREATE node-word
    // order. Multi-origin groups have identical words; no ELEM winner is needed.
    std::vector<unsigned char> seen(input.nodes.size(), 0);
    main.reserve(input.nodes.size());
    for (const auto& face : faces) for (const auto node : face.nodes) {
        Require(node < input.nodes.size(), "Initial surface roster node exceeds physical domain");
        if (!seen[node]) { seen[node] = 1; main.push_back(node); }
    }
    secondary = main;
    // ININT3 calls I25SORS before STI3; all positive ITAB values are explicit.
    std::sort(secondary.begin(), secondary.end(), [&](auto a, auto b) {
        return input.nodes[a].source_id < input.nodes[b].source_id;
    });
    Require(!secondary.empty(), "Original S1 secondary node union is empty");
    secondary_output.swap(secondary);
    main_output.swap(main);
}
}
