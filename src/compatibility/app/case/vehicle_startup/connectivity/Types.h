#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crash::cases::vehicle_startup::connectivity {
enum class Kind : std::uint8_t { Qeph, T3, Qbat, Solid18, Solid24, Solid6z,
    Type13, Type25, PartRoot, PlainGroup, Cin, PointMass,
    Solid18Law44, Solid18Law90, Beam18, SphericalJoint, RevoluteJoint, CylindricalJoint, Count };
enum class Role : std::uint8_t { Constitutive, RigidSkin, Constraint, CoefficientOnly };
inline constexpr std::size_t KindCount = static_cast<std::size_t>(Kind::Count);
inline constexpr std::size_t LegacyKindCount = static_cast<std::size_t>(Kind::Solid18Law44);
constexpr bool IsJoint(Kind kind) noexcept {
    return kind >= Kind::SphericalJoint && kind <= Kind::CylindricalJoint;
}
constexpr bool IsElement(Kind kind) noexcept {
    return kind <= Kind::Type25 || (kind >= Kind::Solid18Law44 && kind <= Kind::Beam18);
}
constexpr std::size_t Width(Kind kind) noexcept {
    constexpr std::size_t widths[]{4,3,4,8,8,6,2,2,0,0,5,1,8,8,2,2,2,2};
    return static_cast<std::size_t>(kind) < KindCount ? widths[static_cast<std::size_t>(kind)] : 0;
}
// Bits Tx,Ty,Tz,Rx,Ry,Rz in the native current joint frame. A graph edge
// describes potential transfer only; these released axes are never locked.
constexpr unsigned ReleasedDofs(Kind kind) noexcept {
    return kind == Kind::SphericalJoint ? 56 : kind == Kind::RevoluteJoint ? 8 :
        kind == Kind::CylindricalJoint ? 9 : 0;
}
// Kind defines the identity namespace. Structural EIDs, TYPE25 WIDs and
// ELEMENT_MASS card IDs are distinct. CIN identity is its retained NSV row;
// source_id/part_id describe its declared mechanical master, not an INCOQ winner.
struct Relation {
    std::uint64_t source_id = 0, part_id = 0;
    std::uint32_t source_row = 0, slot_offset = 0, slot_count = 0;
    Kind kind = Kind::Qeph;
    Role role = Role::Constitutive;
    std::uint16_t rigid_root_index = UINT16_MAX;
};
static_assert(sizeof(Relation) == 32);
enum NodeRole : std::uint8_t { ElementIncidence = 1, RigidSkin = 2,
    ConstraintSupport = 4, PointMass = 8 };
struct Counts {
    std::size_t nodes = 0, relations = 0, slots = 0;
    std::size_t element_components = 0, transfer_components = 0;
    std::size_t rigid_skin_parents = 0, mass_without_element_incidence = 0;
    std::array<std::size_t,KindCount> by_kind{};
};
struct Data {
    // Domain index order, with deterministic minimum original NID labels.
    std::vector<std::uint64_t> element_label, transfer_label;
    std::vector<std::uint8_t> node_roles;
    std::vector<Relation> relations;
    // Original ordered slots, including the repeated T3 CIN slot. No positions.
    std::vector<std::uint32_t> slots;
    Counts counts;
};
enum class Obligation { Pending, PreparedSourceOperators };
struct Limits {
    std::size_t nodes = 524288, relations = 524288, slots = 4*1024*1024;
    std::size_t extra_bytes = 64u << 20, report_bytes = 32u << 20;
    std::size_t total_bytes = std::size_t{8} << 30;
};
struct Forecast {
    std::size_t retained_source_bound = 0, owned_bytes = 0, component_scratch_bytes = 0;
    std::size_t report_reservation = 0, extra_bytes = 0, total_bytes = 0;
    Counts extents;
};
const char* Name(Kind) noexcept;
} // namespace crash::cases::vehicle_startup::connectivity
