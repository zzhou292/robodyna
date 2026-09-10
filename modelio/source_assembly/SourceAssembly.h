#pragma once
#include "SourceAssemblyData.h"
#include <filesystem>
#include <memory>

namespace crash::modelio::assembly {
// Authenticated immutable source declarations, not an accepted mechanics state.
// Read authenticates explicit expected bytes before parsing once. Failures throw
// and publish nothing. Copying shares immutable ownership; moving preserves all
// borrowed views held by other copies. No parser DOM or runtime owner is exposed.
class SourceAssembly {
  public:
    static SourceAssembly Read(const std::filesystem::path&, const ArtifactIdentity&, ReadLimits = {});
    const Data& data() const;
  private:
    explicit SourceAssembly(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
}  // namespace crash::modelio::assembly
