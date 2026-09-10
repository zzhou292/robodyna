#include "SourceAssembly.h"
#include "JsonReader.h"
#include <algorithm>

namespace crash::modelio::assembly {
namespace {
using namespace reader;
void CheckLimits(const ReadLimits& limit) {
    const ReadLimits maximum;
    Require(limit.bytes && limit.bytes <= maximum.bytes && limit.nodes && limit.nodes <= maximum.nodes &&
        limit.parents && limit.parents <= maximum.parents && limit.parts && limit.parts <= maximum.parts &&
        limit.tables && limit.tables <= maximum.tables && limit.curve_points >= 2 && limit.curve_points <= maximum.curve_points &&
        limit.groups && limit.groups <= maximum.groups && limit.group_members >= 3 && limit.group_members <= maximum.group_members &&
        limit.external_nodes && limit.external_nodes <= maximum.external_nodes && limit.spotwelds && limit.spotwelds <= maximum.spotwelds &&
        limit.external_parts && limit.external_parts <= maximum.external_parts,
        "Source assembly limits exceed the bounded reader domain");
}
void CheckIdentity(const ArtifactIdentity& expected,const ReadLimits& limits) {
    reader::Require(expected.bytes && expected.bytes <= limits.bytes && expected.sha256.size() == 64 &&
        std::all_of(expected.sha256.begin(), expected.sha256.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }),
        "Source assembly requires explicit expected content identity within read cap");
}
void ReadScope(const Value& document, Data& data) {
    data.schema = Text(document, "schema");
    Require(data.schema == InventorySchema || data.schema == Law44InventorySchema || data.schema == SectionInventorySchema, "Unsupported assembly schema");
    ReadMaterialPolicy(document,data);
    for (const auto* key : {"simulation_ready", "geometry_modified", "mechanics_capacity_changed",
                            "full_attachment_closure_qualified", "source_mass_equivalence_qualified"}) Flag(document, key, false);
    const auto& source = Member(document, "source");
    data.member_sha256 = Text(source, "member_sha256"); data.archive_sha256 = Text(source, "archive_sha256");
    TextIs(source, "member_sha256", "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301");
    TextIs(source, "archive_sha256", "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451");
    Require(Unsigned(source, "member_bytes") == 42846753, "Source deck byte identity changed");
    const auto& policy = Member(document, "donor_policy");
    TextIs(policy, "revision", "a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(policy, "source_sha256", "87ce68f7e6d9d9186bb2ed5bbc74d1ca97226b8ebaffd48408e2becaae46852d");
    data.donor_revision = Text(policy, "revision"); data.donor_source_sha256 = Text(policy, "source_sha256");
    Flag(policy, "assembly_mechanics_qualified", false); Flag(policy, "ls_dyna_formulation_equivalence_qualified", false);
    Require(Unsigned(policy, "Ishell") == 24 && Unsigned(policy, "Ish3n") == 2 && Unsigned(policy, "NIP") == 3 &&
        Unsigned(policy, "Ismstr") == 2 && Unsigned(policy, "ITHICK") == 1 && Unsigned(policy, "IPLAS") == 1,
        "Unqualified assembly donor policy");
    Require(Ids(Member(policy, "source_elforms"), 2, true) == std::vector<SourceId>{2, 16}, "Source ELFORM policy changed");
    const auto& ledger = Member(document, "native_mass_ledger");
    Flag(ledger, "additional_mass_assigned", false); Flag(ledger, "frontier_only_nodes_are_physical_owner_nodes", false);
    Flag(ledger, "selected_parts_complete", true); TextIs(ledger, "status", "pending shell startup");
    const auto& order = Member(document, "binding_order");
    TextIs(order, "nodes", "ascending source NID"); TextIs(order, "parts", "ascending source PID");
    TextIs(order, "parents", "part order, then original canonical record order within each part");
    TextIs(order, "families", "same parent traversal, separate zero-based QEPH and T3 counters");
    TextIs(order, "tables", "ascending source material, section and curve IDs");
}
void CheckCounts(const Value& document, const Data& data) {
    const auto& counts = reader::Member(document, "counts");
    const auto count = [&](const char* key, std::size_t expected) {
        reader::Require(reader::Unsigned(counts, key) == expected, "Assembly declared count disagrees with complete inventory");
    };
    count("nodes", data.nodes.size()); count("shells", data.parents.size()); count("parts", data.parts.size());
    count("materials", data.materials.size()); count("sections", data.sections.size()); count("curves", data.curves.size());
    count("q4", data.qeph_count); count("native_t3", data.t3_count);
    const auto internal = std::count_if(data.nodal_rigid_groups.begin(), data.nodal_rigid_groups.end(), [](const auto& g) { return g.internal; });
    count("internal_nodal_rigid_groups", internal); count("outgoing_nodal_rigid_groups", data.nodal_rigid_groups.size() - internal);
    count("internal_spotwelds", data.internal_spotwelds.size());
    count("outgoing_spotwelds", data.released_spotwelds.size());
    count("external_nodes", data.boundary.external_node_ids.size());
    count("shared_nodes", reader::Member(reader::Member(document, "geometry"), "shared_node_ids").Size());
    std::vector<bool> materials(data.materials.size()), sections(data.sections.size()), curves(data.curves.size());
    for (const auto& parent : data.parents) {
        materials[parent.material_index] = true; sections[parent.section_index] = true;
        if (parent.curve_index != NoCurveIndex) curves[parent.curve_index] = true;
    }
    for (const auto* used : {&materials, &sections, &curves})
        reader::Require(std::all_of(used->begin(), used->end(), [](bool present) { return present; }), "Unreferenced assembly declaration");
}
}  // namespace
SourceAssembly SourceAssembly::Read(const std::filesystem::path& path, const ArtifactIdentity& expected, ReadLimits limits) {
    CheckLimits(limits);
    CheckIdentity(expected,limits);
    return ReadBytes(output::ReadBounded(path, expected.bytes), expected, limits);
}
SourceAssembly SourceAssembly::ReadBytes(const std::string& bytes, const ArtifactIdentity& expected, ReadLimits limits) {
    CheckLimits(limits);
    CheckIdentity(expected,limits);
    reader::Require(bytes.size() == expected.bytes && output::Sha256(bytes) == expected.sha256,
                    "Source assembly content authentication failed");
    auto candidate = std::make_shared<Data>();
    candidate->authenticated_bytes = bytes;
    candidate->identity = expected;
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag | rapidjson::kParseValidateEncodingFlag>(
        candidate->authenticated_bytes.data(), candidate->authenticated_bytes.size());
    reader::Require(!document.HasParseError() && document.IsObject(), "Invalid source assembly JSON");
    reader::UniqueKeys(document);
    ReadScope(document, *candidate);
    reader::ReadDeclarations(document, limits, *candidate);
    reader::ReadGeometry(document, limits, *candidate);
    reader::ReadAttachments(document, limits, *candidate);
    CheckCounts(document, *candidate);
    return SourceAssembly(std::move(candidate));
}
const Data& SourceAssembly::data() const {
    reader::Require(bool(data_), "Source assembly was moved from");
    return *data_;
}
}  // namespace crash::modelio::assembly
