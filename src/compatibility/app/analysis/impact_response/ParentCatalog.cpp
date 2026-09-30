#include "ParentCatalog.h"

#include "output/BoundedArrayIO.h"
#include "output/full_shell/static_bundle/MappingArrays.h"

namespace crash::analysis::impact_response {

std::vector<ParentCatalogEntry> BuildParentCatalog(
    const output::full_shell::source::PreparedSourceMapping& mapping,
    const output::full_shell::Context& context) {
    namespace arrays = output::arrays;
    namespace source = output::full_shell::source;
    namespace mapping_arrays = output::full_shell::source::detail;
    using output::Require;

    Require(mapping.nodes() == context.nodes() &&
            mapping.parents().size() == context.parents().size() &&
            mapping.digest() == context.identity().source_mapping_sha256,
        "Impact analysis mapping differs from the authenticated frame context");

    const auto& encoded = mapping.arrays();
    const auto node_ids = arrays::Decode<std::uint64_t>(
        encoded[mapping_arrays::NodeIds].descriptor,
        encoded[mapping_arrays::NodeIds].bytes);
    const auto parent_ids = arrays::Decode<std::uint64_t>(
        encoded[mapping_arrays::ParentIds].descriptor,
        encoded[mapping_arrays::ParentIds].bytes);
    const auto references = arrays::Decode<std::uint32_t>(
        encoded[mapping_arrays::ParentReference].descriptor,
        encoded[mapping_arrays::ParentReference].bytes);
    const auto points = arrays::Decode<std::uint32_t>(
        encoded[mapping_arrays::ParentPoints].descriptor,
        encoded[mapping_arrays::ParentPoints].bytes);
    const auto nodes = arrays::Decode<std::uint32_t>(
        encoded[mapping_arrays::ParentNodes].descriptor,
        encoded[mapping_arrays::ParentNodes].bytes);

    const auto count = context.parents().size();
    Require(node_ids.size() == context.nodes() && parent_ids.size() == 4 * count &&
            references.size() == 3 * count && points.size() == 3 * count &&
            nodes.size() == 4 * count,
        "Impact analysis source mapping arrays have inconsistent extents");

    std::vector<ParentCatalogEntry> result;
    result.reserve(count);
    const auto& canonical = mapping.source().data();
    for (std::size_t i = 0; i < count; ++i) {
        const auto& parent = context.parents()[i];
        const auto& part = source::FindPart(canonical, parent.source_part);
        const auto applicability =
            static_cast<output::full_shell::PlasticField>(points[3 * i + 2]);
        Require(parent_ids[4 * i] == parent.source_element &&
                parent_ids[4 * i + 1] == parent.source_part &&
                parent_ids[4 * i + 2] == part.material &&
                parent_ids[4 * i + 3] == part.section &&
                part.shell_section && part.source_elform == parent.source_elform &&
                points[3 * i] == parent.source_elform &&
                points[3 * i + 1] == parent.native_points &&
                applicability == parent.plastic &&
                references[3 * i + 1] == parent.native_family,
            "Impact analysis parent row differs from original source/catalog authority");

        ParentCatalogEntry entry;
        entry.source_element = parent.source_element;
        entry.source_part = parent.source_part;
        entry.source_material = part.material;
        entry.source_section = part.section;
        entry.source_elform = parent.source_elform;
        entry.native_family = parent.native_family;
        entry.family_index = references[3 * i + 2];
        entry.canonical_parent = references[3 * i];
        entry.native_points = parent.native_points;
        entry.plastic = parent.plastic;
        for (unsigned slot = 0; slot < 4; ++slot) {
            const auto node = nodes[4 * i + slot];
            Require(node < node_ids.size(),
                "Impact analysis parent references an absent mapped node");
            entry.mapped_nodes[slot] = node;
            entry.source_nodes[slot] = node_ids[node];
        }
        Require((entry.mapped_nodes[2] == entry.mapped_nodes[3]) ==
                (entry.source_nodes[2] == entry.source_nodes[3]),
            "Impact analysis mapped/source triangle representation differs");
        result.push_back(entry);
    }
    return result;
}

}  // namespace crash::analysis::impact_response
