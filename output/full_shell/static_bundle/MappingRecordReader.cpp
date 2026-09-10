#include "MappingRecords.h"
#include "MappingFields.h"
#include "MappingArrays.h"
#include "InputChecks.h"

namespace crash::output::full_shell::source {
PreparedSourceMapping ReadMappingRecord(const std::filesystem::path& root, const CanonicalSource& source,
        const RecordFile& file, const std::string& expected_digest) {
    const auto bytes = detail::ReadFile(root, file, MappingMetadataByteCap);
    const auto doc = array_json::Parse(bytes, MappingMetadataByteCap);
    const auto descriptors = detail::ParseMappingDocument(source, doc, expected_digest);
    std::array<NamedArray, 8> entries;
    const arrays::Limits cap{source.data().limits.file_bytes, UINT32_MAX, 64};
    for (std::size_t i = 0; i < entries.size(); ++i) {
        Require(descriptors[i].file != file.file, "Mapping metadata aliases a data array");
        entries[i].name = detail::MappingSpecs()[i].name;
        entries[i].descriptor = descriptors[i];
        entries[i].bytes = arrays::ReadBytes(root, descriptors[i], cap);
    }
    Require(MappingDigest(entries, cap) == expected_digest, "Mapping content digest differs from caller authority");
    const auto& nodes = entries[detail::NodeCanonical];
    const auto& parent_reference = entries[detail::ParentReference];
    const auto& points = entries[detail::ParentPoints];
    const auto canonical_nodes = arrays::Decode<std::uint32_t>(nodes.descriptor, nodes.bytes, cap);
    const auto references = arrays::Decode<std::uint32_t>(parent_reference.descriptor, parent_reference.bytes, cap);
    const auto native_points = arrays::Decode<std::uint32_t>(points.descriptor, points.bytes, cap);
    std::vector<NativeParent> parents;
    parents.reserve(source.data().retained_shells);
    for (std::size_t i = 0; i < source.data().retained_shells; ++i) {
        Require(native_points[3 * i + 2] <= static_cast<unsigned>(PlasticField::Unavailable),
            "Unknown stored native point applicability");
        parents.push_back({references[3 * i], references[3 * i + 1], references[3 * i + 2],
            native_points[3 * i + 1], static_cast<PlasticField>(native_points[3 * i + 2])});
    }
    auto mapping = PreparedSourceMapping::Prepare(source,
        {canonical_nodes.data(), canonical_nodes.size(), parents.data(), parents.size()});
    // Regeneration shares the same semantic authority: source IDs/MID/SECID,
    // topology and point applicability cannot be replaced by rehashed records.
    Require(mapping.digest() == expected_digest, "Stored mapping is inconsistent with original source semantics");
    return mapping;
}
} // namespace crash::output::full_shell::source
