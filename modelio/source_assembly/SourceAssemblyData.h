#pragma once
#include "lib_src/math/Fixed3.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace crash::modelio::assembly {
using SourceId = std::uint64_t;
struct ArtifactIdentity { std::size_t bytes = 0; std::string sha256; };
inline ArtifactIdentity PinnedYarisSixPartInventory() {
    return {1731843, "afbc9cc6b9cbbceec766e1aa106b548fcc7468ce1a0d902d5d7b69afb1873d00"};
}
inline ArtifactIdentity PinnedYarisSevenPartInventory() {
    return {1865263, "5ca793e0a2602585f5f789652a9af4d3918e27fd8c029aaad924f792922a797e"};
}
inline constexpr const char* InventorySchema = "robo-dyna.source-assembly-inventory.v1";
inline constexpr const char* Law44InventorySchema = "robo-dyna.source-assembly-inventory.v2";
inline constexpr const char* SectionInventorySchema = "robo-dyna.source-assembly-inventory.v3";
inline constexpr std::size_t NoCurveIndex = SIZE_MAX;
struct ReadLimits {
    std::size_t bytes = 4 * 1024 * 1024, nodes = 2048, parents = 1024, parts = 8;
    std::size_t tables = 8, curve_points = 1024, groups = 64, group_members = 256; // curve_points is the TOTAL pool.
    std::size_t external_nodes = 256, spotwelds = 128;
    std::size_t external_parts = 32; // Frontier declarations are not active parts.
};
struct SourceBlock {
    std::string filename, keyword, raw_text, sha256;
    std::size_t first_line = 0, last_line = 0;
};
struct DeclarationCard {
    std::size_t source_line = 0;
    unsigned blank_mask = 0;
    std::string raw_text;
    std::vector<std::string> names;
    std::vector<std::optional<double>> values;
};
struct Node {
    SourceId source_id = 0;
    std::size_t canonical_index = 0, source_line = 0;
    unsigned blank_mask = 0;
    std::array<unsigned, 2> codes{};
    tl::math::Vec3 position_m;
};
enum class ShellFamily { Qeph, T3 };
struct Parent {
    SourceId source_id = 0, part_id = 0, material_id = 0, section_id = 0, curve_id = 0;
    std::size_t index = 0, part_index = 0, part_parent_index = 0, family_index = 0;
    std::size_t material_index = 0, section_index = 0, curve_index = 0;
    std::size_t canonical_index = 0, source_line = 0;
    unsigned arity = 0, source_elform = 0, blank_mask = 0;
    ShellFamily family = ShellFamily::Qeph;
    std::array<SourceId, 6> raw_record{};
    std::array<std::size_t, 4> nodes{}, canonical_nodes{}, part_local_nodes{};
};
struct Part {
    SourceId id = 0, material_id = 0, section_id = 0;
    std::string title, canonical_manifest_sha256;
    std::size_t first_parent = 0, parent_count = 0;
    std::vector<std::size_t> nodes;
    SourceBlock source;
    std::vector<DeclarationCard> cards;
};
enum class MaterialLaw { LayeredLaw44, LayeredLaw1 };
enum class MaterialHardening { TabulatedLaw44, LinearLaw44 };
struct Material {
    SourceId id = 0, curve_id = 0;
    double density_kg_m3 = 0, young_pa = 0, poisson_ratio = 0;
    double rate_c_per_s = 0, rate_p = 0;
    unsigned source_rate_type = 0;
    std::optional<double> supplied_sigy_pa, supplied_etan_pa;
    SourceBlock source;
    std::vector<DeclarationCard> cards;
    MaterialHardening hardening = MaterialHardening::TabulatedLaw44;
    MaterialLaw law = MaterialLaw::LayeredLaw44;
};
struct Section {
    SourceId id = 0;
    unsigned source_elform = 0, through_thickness_points = 0;
    std::array<double, 4> thickness_m{};
    SourceBlock source;
    std::vector<DeclarationCard> cards;
};
struct Curve {
    SourceId id = 0;
    std::vector<double> plastic_strain, stress_pa;
    SourceBlock source;
    std::vector<DeclarationCard> cards;
};
struct RawAttachmentCard {
    std::size_t source_line = 0;
    unsigned blank_mask = 0;
    std::string raw_text;
    std::vector<std::string> names, fields;
};
struct PartMembership { SourceId part_id = 0; std::vector<SourceId> node_ids; };
struct NodalRigidGroup {
    SourceId id = 0, node_set_id = 0;
    bool internal = false;
    SourceBlock rigid_source, node_set_source;
    std::vector<RawAttachmentCard> rigid_cards, node_set_cards;
    std::vector<SourceId> members, external_nodes;
    std::vector<std::size_t> selected_global_nodes;
    std::vector<PartMembership> selected_membership;
};
// Literal source record only; resolving a record does not supply its mechanics.
struct Spotweld {
    SourceId id = 0;
    std::array<SourceId, 2> node_ids{};
    std::string filename;
    std::size_t keyword_line = 0;
    std::vector<RawAttachmentCard> cards;
    std::vector<SourceId> external_nodes;
    std::vector<PartMembership> selected_membership;
};
using ReleasedSpotweld = Spotweld;
struct InternalSpotweld {
    Spotweld record;
    std::array<std::size_t, 2> nodes{};  // Complete selected endpoint order.
};
struct ReleasedTiedScope {
    std::string filename, reason;
    std::size_t source_line = 0;
    std::vector<SourceId> selected_master_parts, selected_slave_parts;
};
struct AuxiliaryPointMass {
    SourceId source_element_id=0,source_node_id=0;
    std::size_t source_block_line=0,source_card_index=0;
    double supplied_mass_source=0,supplied_mass_kg=0;
};
struct AuxiliarySphericalJoint {
    SourceId source_joint_id=0;
    std::array<SourceId,2> source_node_ids{};
    std::size_t source_block_line=0;
};
// Literal released evidence; no selected owner index or mass contribution.
struct ReleasedAuxiliaryFrontier {
    std::vector<SourceId> source_node_ids;
    std::vector<SourceBlock> source_blocks;
    std::vector<AuxiliaryPointMass> point_masses;
    std::vector<AuxiliarySphericalJoint> spherical_joints;
};
struct ReleasedBoundary {
    std::string policy, interpretation, unresolved_tied_scope;
    std::vector<SourceId> nodal_rigid_ids, spotweld_ids, external_node_ids, external_part_ids;
    std::vector<ReleasedTiedScope> tied_scopes;
    ReleasedAuxiliaryFrontier auxiliary;
};
struct SourceUnits {
    double mass_to_kg = 0, length_to_m = 0, time_to_s = 0;
};
struct Data {
    ArtifactIdentity identity;
    // Original bytes preserve all evidence beyond the typed execution inputs,
    // including array hashes, frontier incidence, unresolved records and cards.
    std::string authenticated_bytes, schema, member_sha256, archive_sha256;
    std::string donor_revision, donor_source_sha256;
    SourceUnits units;
    std::vector<Node> nodes;
    std::vector<Parent> parents;
    std::vector<Part> parts;
    std::vector<Material> materials;
    std::vector<Section> sections;
    std::vector<Curve> curves;
    std::vector<NodalRigidGroup> nodal_rigid_groups;
    std::vector<ReleasedSpotweld> released_spotwelds;
    std::vector<InternalSpotweld> internal_spotwelds;
    ReleasedBoundary boundary;
    std::size_t qeph_count = 0, t3_count = 0;
};
}  // namespace crash::modelio::assembly
