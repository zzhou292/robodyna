#pragma once
#include "modelio/rigid_part/point_mass/Source.h"
#include "modelio/solid_source/VehicleSolidSource.h"
#include "modelio/type13/SourceType13.h"
#include "modelio/beam18/Source.h"

namespace crash::modelio::physical_scope {
namespace source = output::full_shell::source;
namespace rigid = vehicle::rigid_part;
using SourceId = assembly::SourceId;
enum Role : std::uint16_t {
    Shell = 1, Type13Endpoint = 2, Solid = 4, RetainedPointMass = 8,
    ProvisionalType25 = 16, ExcludedTireShell = 32, BeamOrientation = 64, Beam18Endpoint = 128
};
inline constexpr std::uint16_t PhysicalRoles = Shell | Type13Endpoint | Solid | RetainedPointMass | Beam18Endpoint;
inline constexpr std::uint16_t KnownRoles = PhysicalRoles | ProvisionalType25 | ExcludedTireShell | BeamOrientation;
enum class Coverage { None, Partial, Complete };
enum class Family { Shell, Beam, Solid };
struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t nodes = 524288, groups = 1024, members = 32768;
    std::size_t spotwelds = 4096, evidence_nodes = 32768, incidences = 262144;
};
struct Member {
    SourceId node = 0;
    std::uint16_t roles = 0;
};
struct Group {
    SourceId id = 0, node_set_id = 0, child_part_id = 0;
    std::size_t source_index = SIZE_MAX; // Tied source for plain groups; body index for PART roots.
    std::vector<Member> members; // Exact supplied source/topology order.
    Coverage before = Coverage::None, after = Coverage::None;
    std::size_t covered_before = 0, covered_after = 0, tire_members = 0;
};
struct Spotweld {
    SourceId id = 0;
    std::array<SourceId, 2> nodes{};
    std::size_t source_index = 0, first_card = 0;
    // Only literal blank defaults are classified. No property/M/J admission.
    bool default_only = false;
};
struct Incidence {
    SourceId element = 0, part = 0;
    std::uint32_t canonical_row = 0;
    std::uint16_t local_slots = 0;
    Family family = Family::Shell;
    bool selected = false, orientation_only = false;
};
struct NodeEvidence {
    SourceId node = 0;
    std::uint16_t roles = 0;
    std::vector<Incidence> incidence; // Full original source incidence, including excluded parts.
};
struct PointMass {
    SourceId element = 0, node = 0;
    std::size_t source_record = 0, root_index = SIZE_MAX;
    std::uint16_t roles = 0;
};
struct RigidSkin {
    SourceId element = 0, part = 0;
    std::size_t canonical_row = 0, root_index = SIZE_MAX;
};
struct Counts {
    std::size_t baseline_nodes = 0, with_type25_nodes = 0, type25_added_nodes = 0;
    std::size_t role_nodes[8]{};
    std::size_t plain_complete_before = 0, plain_complete_after = 0;
    std::size_t roots_complete_before = 0, roots_complete_after = 0;
    std::size_t optional_spotwelds = 0, outside_mass_in_baseline = 0, outside_mass_with_type25 = 0;
};
struct Forecast {
    // The existing inclusive rigid/point-mass reservation is deliberately
    // conservative; retired parsing is not represented as owned payload.
    std::size_t source_reservation = 0, additional_retained = 0;
    std::size_t workspace = 0, result_reservation = 0, total_bytes = 0;
};
struct Data {
    std::vector<std::uint16_t> node_roles; // Canonical node order; no copied coordinates.
    std::vector<Group> plain_groups, part_roots;
    std::vector<Spotweld> spotwelds;
    std::vector<NodeEvidence> evidence;
    std::vector<PointMass> point_masses;
    std::vector<RigidSkin> rigid_skin; // Original geometry/source association, no material execution role.
    Counts counts;
    std::size_t owned_payload_bytes = 0;
};
// Source-level coverage only. A node hit does not authenticate a complete
// coefficient ledger, DOF inventory, rigid admission, or connection mechanics.
class PhysicalScope {
  public:
    static Forecast Preflight(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
                              const type13::SourceType13&, const solid_source::VehicleSolidSource&, Limits = {});
    static PhysicalScope Prepare(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
                                 const type13::SourceType13&, const solid_source::VehicleSolidSource&, Limits = {});
    // Complete support composition: all4980 selected solids plus all142
    // structural beams. The retained sources supply the named authority.
    static Forecast PreflightVehicleSupports(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
        const type13::SourceType13&, const solid_source::VehicleSolidSource&, const beam18::Source&, Limits = {});
    static PhysicalScope PrepareVehicleSupports(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
        const type13::SourceType13&, const solid_source::VehicleSolidSource&, const beam18::Source&, Limits = {});
    const beam18::Source* structural_beam_source() const noexcept;
    const Data& data() const noexcept;
    const Forecast& forecast() const noexcept;
    const rigid::point_mass::Source& point_mass_source() const noexcept;
    const tied_shell::TiedShellDeclaration& tied_source() const noexcept;
    const type13::SourceType13& type13_source() const noexcept;
    const solid_source::VehicleSolidSource& solid_source() const noexcept;
  private:
    static Forecast PreflightImpl(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
        const type13::SourceType13&, const solid_source::VehicleSolidSource&, const beam18::Source*, Limits);
    static PhysicalScope PrepareImpl(const rigid::point_mass::Source&, const tied_shell::TiedShellDeclaration&,
        const type13::SourceType13&, const solid_source::VehicleSolidSource&, const beam18::Source*, Limits);
    struct Storage;
    explicit PhysicalScope(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
// Bounded report value; callers own file publication and create-only policy.
std::string ReportJson(const PhysicalScope&);
} // namespace crash::modelio::physical_scope
