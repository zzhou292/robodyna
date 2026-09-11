#include "Internal.h"
#include <map>

namespace crash::modelio::tied_shell::classification_detail {
namespace {
using Key = std::pair<std::string, std::size_t>;
using Evidence = std::map<Key, const SourceEvidence*>;
void Index(Evidence& index, const std::vector<SourceEvidence>& sources) {
    for (const auto& source : sources) {
        const auto inserted = index.emplace(Key{source.block.filename, source.block.first_line}, &source);
        if (!inserted.second)
            Require(inserted.first->second->block.sha256 == source.block.sha256,
                    "Conflicting original classification evidence");
    }
}
ClassificationSourceRole Role(const std::string& key) {
    if (key == "*CONSTRAINED_NODAL_RIGID_BODY" || key == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE" ||
        key == "*CONSTRAINED_EXTRA_NODES_SET" || key == "*CONSTRAINED_RIGID_BODIES")
        return ClassificationSourceRole::RigidMembersOutsideObservedSlaves;
    if (key == "*CONSTRAINED_JOINT_CYLINDRICAL_ID" || key == "*CONSTRAINED_JOINT_REVOLUTE_ID" ||
        key == "*CONSTRAINED_JOINT_SPHERICAL_ID")
        return ClassificationSourceRole::SpringJointOutsideObservedSlaves;
    if (key == "*CONSTRAINED_SPOTWELD_ID") return ClassificationSourceRole::ConvertedSpring;
    if (key == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE") return ClassificationSourceRole::SelectedType2;
    if (key == "*CONTACT_AUTOMATIC_SINGLE_SURFACE" || key == "*CONTACT_INTERIOR")
        return ClassificationSourceRole::NonType2Contact;
    if (key.rfind("*RIGIDWALL_", 0) == 0) return ClassificationSourceRole::ReplacedOriginalWall;
    if (key == "*ELEMENT_SOLID") return ClassificationSourceRole::LinearSolidTopology;
    throw std::runtime_error("Unresolved source role can affect tied classification: " + key);
}
bool ConsumedRole(const std::string& key) {
    return key.rfind("*CONSTRAINED_", 0) == 0 || key.rfind("*BOUNDARY_", 0) == 0 ||
        key.rfind("*CONTACT_", 0) == 0 || key.rfind("*RIGIDWALL_", 0) == 0 ||
        key.rfind("*DATABASE_CROSS_SECTION", 0) == 0 || key.find("CYCLIC") != std::string::npos ||
        key.rfind("*ELEMENT_SOLID", 0) == 0;
}
}
void SourceRoles(const source::CanonicalData& canonical, const Data& declaration,
        const AuxiliaryData& auxiliary, const vehicle::rigid_part::SourceData& rigid,
        ClassificationSourceReceipt& receipt, ClassificationSourceLimits limits) {
    Require(auxiliary.wall_policy == OriginalWallPolicy::ReplaceWithMeshWall,
            "Original primitive walls require explicit mesh-wall replacement");
    Evidence evidence;
    Index(evidence, declaration.sources);
    Index(evidence, auxiliary.sources);
    Index(evidence, rigid.sources);
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
        rapidjson::kParseValidateEncodingFlag>(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid classification source inventory");
    const auto& files = Member(document, "source_files");
    Require(files.IsObject() && files.MemberCount() <= limits.source_files,
            "Classification source file cap exceeded");
    std::size_t main_groups = 0, auxiliary_groups = 0, joints = 0, rigid_materials = 0;
    for (auto file = files.MemberBegin(); file != files.MemberEnd(); ++file) {
        const std::string filename(file->name.GetString(), file->name.GetStringLength());
        const auto& blocks = Array(file->value, "blocks", limits.source_blocks);
        Require(blocks.Size() <= limits.source_blocks - receipt.source_blocks, "Classification source block cap exceeded");
        receipt.source_blocks += blocks.Size();
        ++receipt.source_files;
        std::map<std::string, std::size_t> actual;
        std::size_t previous = 0;
        for (const auto& block : blocks.GetArray()) {
            const auto key = Text(block, "keyword");
            const auto first = Unsigned(block, "first_line"), last = Unsigned(block, "last_line");
            Require(Text(block, "file") == filename && first > previous && last >= first,
                    "Classification source block order changed");
            previous = last;
            ++actual[key];
            if (key == "*MAT_RIGID" && filename != "wall.key") ++rigid_materials;
            if (!ConsumedRole(key)) continue;
            const auto role = Role(key);
            UnresolvedBlock item{filename, key, Text(block, "source_block_sha256"), first, last};
            output::arrays::CheckHash(item.sha256);
            const bool canonical_solid = role == ClassificationSourceRole::LinearSolidTopology &&
                                         filename == "yaris-coarse-v1l.key";
            if (canonical_solid)
                Require(source::FindArray(canonical, "solids_records").descriptor.layout.columns == 10 &&
                        source::FindArray(canonical, "solids_node_indices").descriptor.layout.columns == 8,
                        "Original canonical solid topology cannot justify absent tetra10 roles");
            const bool raw = role != ClassificationSourceRole::ReplacedOriginalWall &&
                             role != ClassificationSourceRole::NonType2Contact && !canonical_solid;
            if (raw) {
                const auto found = evidence.find({filename, first});
                Require(found != evidence.end() && found->second->block.sha256 == item.sha256 &&
                        found->second->block.last_line == last && found->second->block.keyword == key,
                        "Original classification role lacks complete authenticated cards");
            }
            if (key == "*CONSTRAINED_NODAL_RIGID_BODY" || key == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE") {
                if (filename == "yaris-coarse-v1l.key") ++main_groups;
                else if (filename == auxiliary.filename) ++auxiliary_groups;
                else Require(false, "Rigid-group source file is outside classification profile");
            }
            if (role == ClassificationSourceRole::SpringJointOutsideObservedSlaves) ++joints;
            if (role == ClassificationSourceRole::SelectedType2) ++receipt.original_type2_interfaces;
            if (role == ClassificationSourceRole::ReplacedOriginalWall) ++receipt.replaced_primitive_walls;
            receipt.roles.push_back({std::move(item), role});
        }
        const auto& counts = Member(file->value, "keyword_counts");
        Require(counts.IsObject() && counts.MemberCount() == actual.size(), "Incomplete classification keyword census");
        for (const auto& [key, count] : actual)
            Require(Unsigned(counts, key.c_str()) == count, "Classification source omitted a keyword block");
    }
    Require(main_groups == receipt.plain_groups && auxiliary_groups == receipt.auxiliary_groups &&
            joints == receipt.joints && rigid_materials == receipt.rigid_parts &&
            receipt.original_type2_interfaces == 1 &&
            receipt.replaced_primitive_walls == auxiliary.original_walls.size(),
            "Classification source role coverage changed");
}
}
