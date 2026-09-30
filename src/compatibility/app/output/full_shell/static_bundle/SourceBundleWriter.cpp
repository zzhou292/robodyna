#include "SourceBundleInternal.h"
#include <algorithm>

namespace crash::output::full_shell::source {
RecordFile WriteSourceBundle(const std::filesystem::path& root, const PreparedSourceBundle& bundle) {
    const auto& data = *bundle.data_;
    const auto& d = data.description;
    const auto& source = data.mapping.source().data();
    for (const auto& f : bundle.reservations()) arrays::CheckedPath(root, f.file, false);
    WriteBytes(arrays::CheckedPath(root, d.canonical_manifest.file, false), source.canonical_bytes);
    WriteBytes(arrays::CheckedPath(root, d.scope_report.file, false), source.scope_bytes);
    const arrays::Limits limits{d.file_byte_cap, UINT32_MAX, 64};
    for (const auto& a : source.arrays)
        arrays::WriteBytes(root, a.descriptor.file, a.descriptor.layout, a.bytes, limits);
    for (const auto& c : d.chunks) {
        const auto bytes = data.member_bytes.substr(c.offset, c.record.bytes);
        Require(Sha256(bytes) == c.record.sha256, "Prepared source chunk identity changed");
        WriteBytes(arrays::CheckedPath(root, c.record.file, false), bytes);
    }
    const auto mapping = WriteMappingRecord(root, data.mapping, d.stem + "-mapping");
    Require(detail::SameFile(mapping, d.mapping), "Written mapping differs from prepared bundle");
    // The static descriptor is the last operation. Every copied source/mapping
    // file is measured and authenticated before this static-only publication.
    std::size_t measured = data.descriptor.bytes;
    for (const auto& f : d.files) {
        if(f.bytes)detail::ReadFile(root,f,d.file_byte_cap);
        else {
            const auto a=std::find_if(source.arrays.begin(),source.arrays.end(),[&](const NamedArray& value){return value.descriptor.file==f.file;});
            Require(a!=source.arrays.end() && arrays::ByteCount(a->descriptor.layout,limits)==0 &&
                f.sha256==Sha256({}) && std::filesystem::file_size(arrays::CheckedPath(root,f.file,true))==0,
                "Zero source payload is not a typed empty canonical array");
        }
        detail::AddBytes(measured, f.bytes, StaticReserveBytes);
    }
    Require(measured == d.static_bytes, "Written static payload differs from plan");
    WriteBytes(arrays::CheckedPath(root, data.descriptor.file, false), data.metadata_bytes);
    return data.descriptor;
}
} // namespace crash::output::full_shell::source
