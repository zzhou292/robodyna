#pragma once

#include "ArtifactIO.h"
#include <vector>

namespace crash::output {
inline constexpr std::size_t kArtifactFileCap = 32*1024*1024;
inline constexpr std::size_t kArtifactTotalCap = 256*1024*1024;
inline constexpr std::size_t kArtifactExtendedTotalCap = 1024*1024*1024;
inline constexpr std::size_t kArtifactFrameCap = 1000;
inline constexpr std::size_t kArtifactInventoryCap = 3*kArtifactFrameCap+16;

// Hashes closed create-only files. No filesystem/state ownership or completion
// marker; the case writer stages its manifest after all records are present.
class ArtifactInventory {
  public:
    explicit ArtifactInventory(std::filesystem::path directory, std::size_t total_cap = kArtifactTotalCap);
    void Add(const std::string& basename, std::size_t component_cap = kArtifactFileCap);
    void AppendTo(Document&) const;
    std::size_t bytes() const noexcept { return bytes_; }
  private:
    struct Entry { std::string file, hash; std::size_t bytes; };
    std::filesystem::path directory_;
    std::vector<Entry> entries_;
    std::size_t bytes_ = 0, total_cap_ = kArtifactTotalCap;
};
} // namespace crash::output
