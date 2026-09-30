#include "Internal.h"
#include <charconv>

namespace crash::cases::vehicle_startup::post_kinchk_detail {
TiedPostKinChkReceipt Receipt(const tied::source::CanonicalData& canonical, const tied::Data& declaration,
        const TiedFinalizationReceipt& finalized, const tied::ClassificationSourceReceipt& context) {
    using output::Require;
    Require(finalized.phase == TiedFinalizationPhase::FreshPhysicalCoefficientsBeforeIniend &&
            finalized.is1 == 2 && finalized.level == 28 && finalized.type2_count == 1 &&
            finalized.type2_ordinal == 1 && finalized.contact_source == declaration.contact_source &&
            context.original_type2_interfaces == 1 && context.observed_slaves == declaration.slave_nodes.size(),
            "Post-KINCHK source/caller scope differs");
    Require(declaration.contact_source < declaration.sources.size() && !context.wall.node_ids.empty() &&
            !context.wall.shell_ids.empty() && context.wall.part_id && context.replaced_primitive_walls,
            "Post-KINCHK requires the authenticated whole original wall replacement");
    std::size_t selected = 0, replaced = 0;
    for (const auto& evidence : context.roles) {
        switch (evidence.role) {
        case tied::ClassificationSourceRole::RigidMembersOutsideObservedSlaves:
        case tied::ClassificationSourceRole::SpringJointOutsideObservedSlaves:
        case tied::ClassificationSourceRole::ConvertedSpring:
        case tied::ClassificationSourceRole::NonType2Contact:
        case tied::ClassificationSourceRole::LinearSolidTopology:
            break;
        case tied::ClassificationSourceRole::SelectedType2:
            ++selected;
            break;
        case tied::ClassificationSourceRole::ReplacedOriginalWall:
            ++replaced;
            break;
        default:
            Require(false,"Unresolved converted role can change post-KINCHK state");
        }
    }
    Require(selected == 1 && replaced == context.replaced_primitive_walls,
            "Post-KINCHK exclusions lack complete source role evidence");
    TiedPostKinChkReceipt out;
    const auto& source = declaration.sources[declaration.contact_source].block;
    out.original_contact = {source.filename,source.keyword,source.sha256,source.first_line,source.last_line};
    Require(out.original_contact.keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE" &&
            out.original_contact.first_line && out.original_contact.first_line <= UINT32_MAX,
            "Post-KINCHK original interface association changed");
    const auto& hash = canonical.inputs.source_member.sha256;
    output::arrays::CheckHash(hash);
    const auto parsed = std::from_chars(hash.data(),hash.data()+16,out.source_instance_id,16);
    Require(parsed.ec == std::errc{} && parsed.ptr == hash.data()+16 && out.source_instance_id,
            "Invalid post-KINCHK source association");
    out.native_interface_ordinal = 1;
    out.replaced_wall_part = context.wall.part_id;
    out.native_profile = native_search::KinChkProfile::NoWallRbeOrCyclic;
    out.observed_slaves = context.observed_slaves;
    return out;
}
}
