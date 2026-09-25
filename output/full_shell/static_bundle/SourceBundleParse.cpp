#include "SourceBundleInternal.h"
#include "SourceAuthority.h"
#include <set>

namespace crash::output::full_shell::source::detail {
namespace {
using namespace array_json;
RecordFile ParseFile(const Value& v, std::size_t cap) {
    Keys(v, {"file", "sha256", "bytes"});
    const auto n = UInt(v["bytes"]);
    Require(n <= cap, "Static source file exceeds capacity");
    RecordFile f{Text(v["file"]), Text(v["sha256"]), static_cast<std::size_t>(n)};
    arrays::CheckRelativeName(f.file);
    arrays::CheckHash(f.sha256);
    // Staged inventory only. Zero entries must later match a reconstructed
    // canonical array whose typed dimensions require zero elements.
    Require(f.bytes || f.sha256==Sha256({}),"Empty source array hash differs");
    return f;
}
void ExpectedCopy(const RecordFile& copy, const RecordFile& original, const std::string& path) {
    Require(copy.file == path && copy.bytes == original.bytes && copy.sha256 == original.sha256,
        "Copied source metadata differs from external authority");
}
} // namespace
BundleDescription ParseBundleDocument(const Value& v, const SourceInputs& authority,
        const std::string& digest, const RecordFile& descriptor, SourceLimits limits) {
    arrays::CheckHash(digest);
    CheckUnits(authority.units);
    Keys(v, {"schema", "scope", "stem", "mapping_sha256", "file_byte_cap", "static_payload_bytes",
        "source_authority", "canonical_manifest_file", "scope_report_file", "mapping_file", "member_chunks", "files"});
    Require(Text(v["schema"]) == BundleSchema && Text(v["scope"]) ==
        "static_source_bundle_only_native_admission_and_run_completion_external" &&
        Text(v["mapping_sha256"]) == digest, "Static bundle schema/scope/mapping identity differs");
    CheckAuthority(authority, v["source_authority"]);
    BundleDescription d;
    d.stem = Text(v["stem"]);
    CheckBundleStem(d.stem);
    d.mapping_sha256 = digest;
    const auto cap = UInt(v["file_byte_cap"]), total = UInt(v["static_payload_bytes"]);
    Require(cap && cap <= limits.file_bytes && cap <= kArtifactFileCap && total <= StaticReserveBytes &&
        descriptor.file == d.stem + ".bundle.json" && descriptor.bytes <= cap,
        "Static bundle path or byte limits differ");
    d.file_byte_cap = static_cast<std::size_t>(cap);
    d.static_bytes = static_cast<std::size_t>(total);
    const auto& inventory = v["files"];
    Require(inventory.IsArray() && inventory.Size() >= 29 && inventory.Size() <= 92,
        "Static bundle file inventory exceeds capacity");
    std::set<std::string> names{descriptor.file};
    std::size_t measured = descriptor.bytes;
    for (const auto& f : inventory.GetArray()) {
        auto file = ParseFile(f, d.file_byte_cap);
        Require(names.insert(file.file).second, "Duplicate static source file");
        AddBytes(measured, file.bytes, StaticReserveBytes);
        d.files.push_back(std::move(file));
    }
    Require(measured == d.static_bytes, "Static bundle payload omits or miscounts bytes");
    d.canonical_manifest = FindBundleFile(d, Text(v["canonical_manifest_file"]));
    d.scope_report = FindBundleFile(d, Text(v["scope_report_file"]));
    d.mapping = FindBundleFile(d, Text(v["mapping_file"]));
    ExpectedCopy(d.canonical_manifest, authority.canonical_manifest, d.stem + "-canonical.json");
    ExpectedCopy(d.scope_report, authority.scope_report, d.stem + "-scope.json");
    Require(d.mapping.file == d.stem + "-mapping.mapping.json" && d.mapping.bytes <= MappingMetadataByteCap,
        "Static mapping descriptor path/size differs");
    const auto& chunks = v["member_chunks"];
    Require(chunks.IsArray() && !chunks.Empty() && chunks.Size() <= 64,
        "Static source member chunk count exceeds capacity");
    for (const auto& c : chunks.GetArray()) {
        Keys(c, {"file", "offset"});
        const auto offset = UInt(c["offset"]);
        Require(offset <= authority.source_member.bytes, "Source chunk offset exceeds member extent");
        d.chunks.push_back({static_cast<std::size_t>(offset), FindBundleFile(d, Text(c["file"]))});
    }
    CheckChunks(d.chunks, authority.source_member.bytes, d.file_byte_cap);
    Require(authority.source_member.bytes <= limits.source_member_bytes,
        "Original source exceeds reader member capacity");
    return d;
}
} // namespace crash::output::full_shell::source::detail
