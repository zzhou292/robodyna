#include "SourceBundleInternal.h"
#include <iomanip>
#include <sstream>
#include <set>

namespace crash::output::full_shell::source::detail {
void CheckBundleStem(const std::string& stem) {
    arrays::CheckRelativeName(stem);
    Require(stem.size() <= 80 && stem.find('/') == std::string::npos, "Invalid source bundle stem");
}
bool SameFile(const RecordFile& a, const RecordFile& b) noexcept {
    return a.file == b.file && a.bytes == b.bytes && a.sha256 == b.sha256;
}
RecordFile FindBundleFile(const BundleDescription& d, const std::string& name) {
    for (const auto& f : d.files) if (f.file == name) return f;
    throw std::runtime_error("Missing source bundle file");
}
void CheckChunks(const std::vector<SourceChunk>& chunks, std::size_t bytes, std::size_t cap) {
    Require(cap && cap <= kArtifactFileCap && bytes && bytes <= 64 * 1024 * 1024 &&
        !chunks.empty() && chunks.size() <= 64 && chunks.size() == 1 + (bytes - 1) / cap,
        "Invalid bounded source chunk count");
    std::size_t offset = 0;
    std::set<std::string> names;
    for (const auto& chunk : chunks) {
        arrays::CheckRelativeName(chunk.record.file);
        arrays::CheckHash(chunk.record.sha256);
        Require(chunk.offset == offset && chunk.record.bytes == std::min(cap, bytes - offset) &&
            names.insert(chunk.record.file).second, "Source chunks have gaps, overlap, or incorrect extent");
        offset += chunk.record.bytes;
    }
    Require(offset == bytes, "Incomplete original source member");
}
std::vector<SourceChunk> DescribeChunks(const std::string& stem, const std::string& bytes, std::size_t cap) {
    CheckBundleStem(stem);
    Require(cap && cap <= kArtifactFileCap && !bytes.empty() && bytes.size() <= 64 * 1024 * 1024 &&
        1 + (bytes.size() - 1) / cap <= 64, "Source chunk planning exceeds capacity");
    std::vector<SourceChunk> result;
    for (std::size_t offset = 0; offset < bytes.size();) {
        const auto count = std::min(cap, bytes.size() - offset);
        const auto part = bytes.substr(offset, count);
        std::ostringstream name;
        name << stem << "-key-" << std::setfill('0') << std::setw(6) << result.size() << ".bin";
        result.push_back({offset, {name.str(), Sha256(part), count}});
        offset += count;
    }
    CheckChunks(result, bytes.size(), cap);
    return result;
}
} // namespace crash::output::full_shell::source::detail
