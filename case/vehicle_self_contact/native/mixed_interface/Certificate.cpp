#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
Report SelectedRoleReport(const coated::Inputs& geometry, const coated::Classification& roles) {
    if (roles.contact_complete) return {Status::Ready, "Complete selected source membership"};
    const auto row = roles.first_unready_contact_shell;
    if (row >= geometry.shells.size() || row >= roles.roles.size())
        return {Status::InvalidInput, "Incomplete selected role diagnostic"};
    Report result;
    result.source_element = geometry.shells[row].primary.source_id;
    if (roles.roles[row].matches > 1) {
        result.status = Status::NeedsNativeReaderOrder;
        result.reason = "Selected IN24 membership has multiple solids; authentic first-solid order is required";
    } else {
        result.status = Status::UnsupportedSource;
        result.reason = "Selected unique IN24 orientation has no defined finite source result";
    }
    return result;
}
Report Certify(const coated::Inputs& geometry, const Packed& packed, const coated::Classification& roles,
        const std::vector<std::uint8_t>& flags, const f::Snapshot& result,
        Certificate& published, std::vector<RoleObservation>& observations) {
    Report bad{Status::InvalidInput, "Mixed classification disagrees with complete source"};
    const auto selected = SelectedRoleReport(geometry, roles);
    if (selected.status != Status::Ready) return selected;
    const auto count = packed.faces.size();
    if (roles.roles.size() != geometry.shells.size() || result.raw_face_count != count ||
        packed.raw_shell_to_physical.size() != count || flags.size() != geometry.solids.size() ||
        result.physical_solid_count != flags.size() || !result.classifications || !result.raw_origins ||
        !result.raw_to_primary || !result.primary || !result.identities || !result.primary_to_raw ||
        (!flags.empty() && !result.surface_solid_flags) || !result.primary_count ||
        result.shell_primary_count > result.primary_count ||
        result.main_count != result.primary_count + result.shell_primary_count) return bad;
    Certificate next;
    std::vector<RoleObservation> staged;
    staged.reserve(count);
    std::vector<std::uint32_t> origin_counts(result.primary_count, 0);
    for (std::size_t i = 0; i < count; ++i) {
        bad.raw_face = i;
        const auto& face = packed.faces[i];
        bad.source_element = face.source.element_id;
        const auto& observed = result.classifications[i];
        const auto& origin = result.raw_origins[i];
        const bool solid = face.source.kind == source::ParentKind::Solid;
        if (origin.kind != (solid ? s::PrimaryFaceKind::Solid : s::PrimaryFaceKind::Shell) ||
            origin.physical_parent_id != face.source.element_id || origin.local_face != face.source.solid_face ||
            origin.origin != s::PrimaryOrigin::SingleSourceFace || origin.origin_count != 1 ||
            result.raw_to_primary[i] >= result.primary_count) return bad;
        ++origin_counts[result.raw_to_primary[i]];
        RoleObservation value;
        value.native_role = observed.role;
        if (solid) {
            ++next.raw_solids;
            if (observed.role != 1 || observed.matched_solid != UINT32_MAX ||
                packed.raw_shell_to_physical[i] != UINT32_MAX) return bad;
        } else {
            ++next.raw_shells;
            const auto row = packed.raw_shell_to_physical[i];
            if (row >= geometry.shells.size() || !geometry.shells[row].contact_selected) return bad;
            const auto& role = roles.roles[row];
            const bool ordinary = role.state == coated::RoleState::Ordinary;
            const bool forward = role.state == coated::RoleState::CoatingForward;
            const bool reverse = role.state == coated::RoleState::CoatingReversed;
            const auto wanted = ordinary ? face.raw_role : (reverse ? -(face.raw_role+1) : face.raw_role+1);
            if ((!ordinary && !forward && !reverse) || observed.role != wanted) return bad;
            if (ordinary) {
                if (role.matches || observed.matched_solid != UINT32_MAX) return bad;
                ++next.ordinary_shells;
            } else {
                if (role.matches != 1 || role.first_solid >= geometry.solids.size() ||
                    observed.matched_solid != role.first_solid) return bad;
                value.unique_coating_solid_eid = geometry.solids[role.first_solid].source_id;
                ++next.unique_coatings;
            }
        }
        staged.push_back(value);
    }
    for (std::size_t i = 0; i < flags.size(); ++i)
        if (flags[i] != result.surface_solid_flags[i]) return bad;
    for (std::size_t p = 0; p < result.primary_count; ++p) {
        const auto& identity = result.identities[p];
        const auto winner = result.primary_to_raw[p];
        if (!origin_counts[p] || identity.origin_count != origin_counts[p] || winner >= count ||
            result.raw_to_primary[winner] != p) return bad;
        if (identity.kind == s::PrimaryFaceKind::Solid) ++next.solid_primaries;
        else if (identity.kind == s::PrimaryFaceKind::Shell) ++next.shell_primaries;
        else return bad;
        if (identity.origin == s::PrimaryOrigin::MultipleOrigins) {
            if (origin_counts[p] < 2 || identity.physical_parent_id || identity.local_face || result.primary[p].source_id)
                return bad;
            ++next.multi_origin_primaries;
        } else if (identity.origin != s::PrimaryOrigin::SingleSourceFace || origin_counts[p] != 1 ||
            identity.physical_parent_id != result.raw_origins[winner].physical_parent_id ||
            identity.local_face != result.raw_origins[winner].local_face ||
            result.primary[p].source_id != identity.physical_parent_id) return bad;
    }
    if (next.raw_shells != roles.contact.shells || next.shell_primaries != result.shell_primary_count) return bad;
    if (staged.capacity() > 2*count || origin_counts.capacity() > 2*result.primary_count)
        return {Status::ResourceLimit, "Mixed source certificate capacity exceeds reservation"};
    next.filtered_primaries = result.primary_count;
    next.coalesced_origins = count - result.primary_count;
    next.unique_selected_membership = true;
    next.complete_origins = true;
    next.complete_solid_flags = true;
    published = next;
    observations = std::move(staged);
    return {Status::Ready, "Unique selected membership and every raw source origin are certified"};
}
}
