#include "Values.h"
#include "output/ArtifactIO.h"
#include "lib_src/collision/radioss_type25/startup/CoatingOrientation.h"
#include <algorithm>
#include <climits>
namespace crash::cases::vehicle_self_contact::native::coated {
namespace {
void Count(RoleCounts& counts, const Role& role, bool arithmetic_failure) {
    ++counts.shells;
    output::Require(role.matches <= SIZE_MAX - counts.matches, "Coating membership count overflow");
    counts.matches += role.matches;
    counts.unmatched += role.matches == 0;
    counts.unique += role.matches == 1;
    counts.multiple += role.matches > 1;
    counts.forward += role.state == RoleState::CoatingForward;
    counts.reversed += role.state == RoleState::CoatingReversed;
    counts.arithmetic_failures += arithmetic_failure;
}
}
Classification Classify(const Inputs& input) {
    output::Require(!input.nodes.empty() && input.nodes.size() <= 1048576 &&
        input.shells.size() <= 524288 && input.solids.size() <= 16384,
        "Coating membership source extent exceeds its bounded profile");
    struct Incidence { std::uint32_t node, solid; };
    std::vector<Incidence> incidences;
    incidences.reserve(8 * input.solids.size());
    output::Require(incidences.capacity() <= 16*input.solids.size(), "Membership incidence allocation exceeds reserved capacity");
    for (std::size_t i = 0; i < input.solids.size(); ++i) {
        const auto& solid = input.solids[i];
        output::Require(solid.phase == PacketPhase::ReaderBeforeInitia, "Wrong solid packet phase for coating membership");
        output::Require(solid.kind == ReaderKind::Hex8 || solid.kind == ReaderKind::DeclaredPenta6,
                        "Unknown solid reader packet kind");
        if (solid.kind == ReaderKind::DeclaredPenta6) {
            output::Require(solid.nodes[3] == solid.nodes[0] && solid.nodes[7] == solid.nodes[4],
                            "PENTA contact packet has post-INITIA or incorrect duplicate slots");
            constexpr unsigned active[]{0,1,2,4,5,6};
            for (unsigned a = 0; a < 6; ++a) for (unsigned b = 0; b < a; ++b)
                output::Require(solid.nodes[active[a]] != solid.nodes[active[b]], "PENTA reader active slots repeat");
        }
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto node = solid.nodes[slot];
            output::Require(node < input.nodes.size(), "Coating solid node is outside physical domain");
            if (std::find(solid.nodes.begin(), solid.nodes.begin() + slot, node) == solid.nodes.begin() + slot)
                incidences.push_back({node, std::uint32_t(i)});
        }
    }
    std::sort(incidences.begin(), incidences.end(), [](auto a, auto b) {
        return a.node < b.node || (a.node == b.node && a.solid < b.solid);
    });
    std::vector<std::uint32_t> offsets(input.nodes.size() + 1, 0);
    for (const auto& item : incidences) ++offsets[item.node + 1];
    for (std::size_t i = 1; i < offsets.size(); ++i) offsets[i] += offsets[i-1];
    output::Require(offsets.capacity() <= 2*(input.nodes.size()+1), "Membership offsets exceed reserved capacity");
    Classification result;
    result.roles.resize(input.shells.size());
    output::Require(result.roles.capacity() <= 2*input.shells.size(), "Membership role allocation exceeds reserved capacity");
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& shell = input.shells[i];
        const auto& face = shell.primary;
        output::Require(face.layout == n::ShellLayout::Quad4 || face.layout == n::ShellLayout::Triangle3,
                        "Unsupported physical shell topology in coating membership");
        for (const auto node : face.nodes)
            output::Require(node < input.nodes.size(), "Coating shell node is outside physical domain");
        const bool triangle = face.layout == n::ShellLayout::Triangle3;
        output::Require(!triangle || face.nodes[2] == face.nodes[3], "T3 source must repeat its fourth slot");
        for (unsigned a = 0; a < (triangle ? 3u : 4u); ++a)
            for (unsigned b = 0; b < a; ++b)
                output::Require(face.nodes[a] != face.nodes[b], "Physical shell has repeated active source nodes");
        auto& role = result.roles[i];
        for (auto at = offsets[face.nodes[0]]; at < offsets[face.nodes[0]+1]; ++at) {
            const auto solid = incidences[at].solid;
            output::Require(result.candidate_visits != SIZE_MAX, "Coating candidate visit count overflow");
            ++result.candidate_visits;
            bool all = true;
            for (const auto node : face.nodes)
                all = all && std::find(input.solids[solid].nodes.begin(), input.solids[solid].nodes.end(), node) != input.solids[solid].nodes.end();
            if (!all) continue;
            if (!role.matches) role.first_solid = solid;
            ++role.matches;
        }
        s::Status status = s::Status::Ok;
        if (!role.matches) role.state = RoleState::Ordinary;
        else if (role.matches == 1) {
            const auto& solid = input.solids[role.first_solid];
            s::NativeCoatingOrientationInput packet;
            packet.node_count = 8;
            for (unsigned slot = 0; slot < 8; ++slot) packet.positions[slot] = input.nodes[solid.nodes[slot]].native_position;
            for (unsigned corner = 0; corner < 3; ++corner)
                packet.segment_slots[corner] = unsigned(std::find(solid.nodes.begin(), solid.nodes.end(), face.nodes[corner]) - solid.nodes.begin());
            s::NativeCoatingOrientationResult orientation;
            status = s::EvaluateNativeCoatingOrientation(packet, &orientation);
            if (status == s::Status::Ok) {
                role.determinant = orientation.center_triangle_determinant;
                role.state = orientation.orientation == s::CoatingOrientation::Forward ? RoleState::CoatingForward : RoleState::CoatingReversed;
            }
        } else status = s::Status::UnsupportedProfile; // No invented native first-solid order.
        if (role.state == RoleState::Unresolved && result.first_unready_shell == SIZE_MAX) {
            result.first_unready_shell = i;
            result.first_status = status;
        }
        if (role.state == RoleState::Unresolved && shell.contact_selected && result.first_unready_contact_shell == SIZE_MAX) {
            result.first_unready_contact_shell = i;
            result.first_contact_status = status;
        }
        const bool arithmetic_failure = role.matches == 1 && status != s::Status::Ok;
        Count(result.physical, role, arithmetic_failure);
        if (shell.contact_selected) Count(result.contact, role, arithmetic_failure);
    }
    result.complete = result.first_unready_shell == SIZE_MAX;
    result.contact_complete = result.first_unready_contact_shell == SIZE_MAX;
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
