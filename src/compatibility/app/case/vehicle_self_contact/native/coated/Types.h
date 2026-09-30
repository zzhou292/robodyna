#pragma once
#include "lib_src/collision/radioss_type25/startup/Types.h"
#include <array>
#include <vector>
namespace crash::cases::vehicle_self_contact::native::coated {
namespace s = tlfea::contact::radioss_type25::startup;
namespace n = tlfea::contact::radioss_type25;
enum class ReaderKind { Hex8, DeclaredPenta6 };
enum class PacketPhase { ReaderBeforeInitia };
enum class RoleState { Unresolved, Ordinary, CoatingForward, CoatingReversed };
struct Node {
    std::uint64_t source_id = 0;
    std::uint32_t canonical_row = 0;
    n::Vector native_position;
};
struct Shell {
    s::PrimaryFace primary;
    std::uint64_t part_id = 0;
    std::uint32_t canonical_row = 0, source_line = 0, physical_parent = 0;
    bool contact_selected = false;
};
struct Solid {
    std::uint64_t source_id = 0, part_id = 0;
    std::uint32_t canonical_row = 0, source_line = 0, family = 0, reference_index = 0;
    ReaderKind kind = ReaderKind::Hex8;
    PacketPhase phase = PacketPhase::ReaderBeforeInitia;
    // Complete native reader IXS(2:9), in physical-domain node indices.
    // PENTA repeats remain original reader slots; no post-INITIA map is applied.
    std::array<std::uint32_t, 8> nodes{};
};
struct Inputs {
    std::vector<Node> nodes;
    std::vector<Shell> shells; // Every retained physical parent, source order.
    std::vector<Solid> solids; // Every retained physical solid, source record order.
    n::UnitScale units;
    std::size_t coordinate_roundtrip_changes = 0;
};
struct Role {
    RoleState state = RoleState::Unresolved;
    std::uint32_t matches = 0, first_solid = UINT32_MAX;
    double determinant = 0;
};
struct RoleCounts {
    std::size_t shells = 0, matches = 0, unmatched = 0, unique = 0, multiple = 0;
    std::size_t forward = 0, reversed = 0, arithmetic_failures = 0;
};
struct Classification {
    std::vector<Role> roles;
    RoleCounts physical, contact;
    std::size_t candidate_visits = 0, first_unready_shell = SIZE_MAX;
    s::Status first_status = s::Status::Ok;
    std::size_t first_unready_contact_shell = SIZE_MAX;
    s::Status first_contact_status = s::Status::Ok;
    bool complete = false, contact_complete = false;
};
struct Order {
    // Complete selected roster only. Physical assembly still owns every shell.
    std::vector<std::uint32_t> primary_to_physical;
    std::vector<std::uint32_t> physical_to_primary; // UINT32_MAX for non-contact.
};
} // namespace crash::cases::vehicle_self_contact::native::coated
