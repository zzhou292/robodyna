#include "SourceBundleInternal.h"
#include "SourceAuthority.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::output::full_shell::source::detail {
namespace {
Document FileDocument(const RecordFile& f) {
    Document d; d.SetObject();
    String(d, "file", f.file);
    String(d, "sha256", f.sha256);
    Integer(d, "bytes", f.bytes);
    return d;
}
} // namespace
Document BundleDocument(const SourceInputs& authority, const BundleDescription& s) {
    Document d; d.SetObject();
    String(d, "schema", BundleSchema);
    String(d, "scope", "static_source_bundle_only_native_admission_and_run_completion_external");
    String(d, "stem", s.stem);
    String(d, "mapping_sha256", s.mapping_sha256);
    Integer(d, "file_byte_cap", s.file_byte_cap);
    Integer(d, "static_payload_bytes", s.static_bytes);
    array_json::Child(d, "source_authority", AuthorityDocument(authority));
    String(d, "canonical_manifest_file", s.canonical_manifest.file);
    String(d, "scope_report_file", s.scope_report.file);
    String(d, "mapping_file", s.mapping.file);
    Value chunks(rapidjson::kArrayType), files(rapidjson::kArrayType);
    for (const auto& c : s.chunks) {
        Document item; item.SetObject();
        String(item, "file", c.record.file);
        Integer(item, "offset", c.offset);
        chunks.PushBack(Value(item, d.GetAllocator()), d.GetAllocator());
    }
    for (const auto& f : s.files) {
        auto item = FileDocument(f);
        files.PushBack(Value(item, d.GetAllocator()), d.GetAllocator());
    }
    d.AddMember("member_chunks", chunks, d.GetAllocator());
    d.AddMember("files", files, d.GetAllocator());
    return d;
}
std::string EncodeBundleMetadata(const SourceInputs& authority, BundleDescription& d) {
    std::size_t payload = 0;
    for (const auto& f : d.files) AddBytes(payload, f.bytes, StaticReserveBytes);
    // Only the decimal byte-count field is self-sized; hashes never reference
    // themselves. The bounded fixed point includes every descriptor byte.
    d.static_bytes = payload;
    for (unsigned iteration = 0; iteration < 8; ++iteration) {
        const auto doc = BundleDocument(authority, d);
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        Require(doc.Accept(writer) && buffer.GetSize() <= BundleMetadataByteCap,
            "Static source descriptor exceeds capacity");
        std::size_t total = payload;
        AddBytes(total, buffer.GetSize(), StaticReserveBytes);
        if (total == d.static_bytes) return {buffer.GetString(), buffer.GetSize()};
        d.static_bytes = total;
    }
    throw std::runtime_error("Static source descriptor size did not stabilize");
}
} // namespace crash::output::full_shell::source::detail
