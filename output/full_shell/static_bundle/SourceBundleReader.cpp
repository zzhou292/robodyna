#include "SourceBundleInternal.h"

namespace crash::output::full_shell::source {
PreparedSourceMapping ReadSourceBundle(const std::filesystem::path& root, const RecordFile& descriptor,
        const SourceInputs& authority, const std::string& digest, SourceLimits limits) {
    detail::SourceReadBudget(authority, limits); // Before file reads or member allocation.
    arrays::CheckHash(digest);
    const auto bytes = detail::ReadFile(root, descriptor, std::min(limits.file_bytes, BundleMetadataByteCap));
    const auto document = array_json::Parse(bytes, BundleMetadataByteCap);
    const auto d = detail::ParseBundleDocument(document, authority, digest, descriptor, limits);
    std::string member;
    member.reserve(authority.source_member.bytes);
    for (const auto& chunk : d.chunks) {
        const auto part = detail::ReadFile(root, chunk.record, d.file_byte_cap);
        member.append(part);
    }
    Require(member.size() == authority.source_member.bytes && Sha256(member) == authority.source_member.sha256,
        "Reassembled source differs from original complete member");
    SourceInputs in = authority;
    in.canonical_root = root;
    in.scope_root = root;
    in.canonical_manifest = d.canonical_manifest;
    in.scope_report = d.scope_report;
    limits.file_bytes = d.file_byte_cap;
    auto source = CanonicalSource::ReadWithMemberBytes(in, member, limits);
    std::string().swap(member); // Original key is evidence, not retained runtime state.
    auto mapping = ReadMappingRecord(root, source, d.mapping, digest);
    const auto mapping_plan = PlanMappingRecord(mapping, d.stem + "-mapping");
    detail::CheckInventory(d.files, detail::Inventory(mapping, mapping_plan, d));
    return mapping;
}
} // namespace crash::output::full_shell::source
