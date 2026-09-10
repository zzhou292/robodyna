#pragma once
#include "ActualMappingSupport.h"
#include "../SourceBundleInternal.h"
#include "../../tests/FileWriteLimit.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::output::full_shell::source::test {
namespace ft = crash::output::full_shell::test;
inline BundleRequest BundleOptions() {
    BundleRequest r;
    r.archive.nodes = ActualMapping().nodes();
    r.archive.parents = ActualMapping().parents().size();
    r.archive.plastic_points = 0; // Fixture explicitly declares unavailable, not zero physical PLA.
    r.archive.frames = 88;
    r.archive.fixed_dt = 0x1p-26;
    r.archive.requested_duration = .02;
    r.archive.intervals = 1342178;
    r.archive.static_files = {{"manifest.json", 1024 * 1024}, {"frame-index.json", 256 * 1024},
        {"configuration.json", 8 * 1024 * 1024}, {"placed-wall.json", 1024 * 1024}};
    return r;
}
inline const PreparedSourceBundle& ActualBundle() {
    static const auto bundle = PreparedSourceBundle::Prepare(ActualMapping(), BundleOptions());
    return bundle;
}
inline void BundleDirectories(const std::filesystem::path& root) {
    std::filesystem::create_directory(root / "arrays");
}
inline RecordFile RewriteBundle(const std::filesystem::path& root, const RecordFile& record, const Document& doc) {
    Document changed;
    changed.CopyFrom(doc, changed.GetAllocator());
    std::size_t payload = 0;
    for (const auto& f : changed["files"].GetArray()) payload += f["bytes"].GetUint64();
    for (unsigned iteration = 0; iteration < 8; ++iteration) {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        Require(changed.Accept(writer), "Fixture metadata encoding failed");
        const auto total = payload + buffer.GetSize();
        if (total != changed["static_payload_bytes"].GetUint64()) {
            changed["static_payload_bytes"].SetUint64(total);
            continue;
        }
        const std::string bytes(buffer.GetString(), buffer.GetSize());
        ft::Overwrite(root / record.file, bytes);
        return {record.file, Sha256(bytes), bytes.size()};
    }
    throw std::runtime_error("Fixture metadata size did not stabilize");
}
} // namespace crash::output::full_shell::source::test
