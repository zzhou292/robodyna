#pragma once
#include "Types.h"
namespace crash::modelio::solid_control {
// Authenticated direct declaration only. Property sharing is intentionally a
// later stage, preserving callers that need direct membership before mapping.
class DirectSource {
  public:
    static DirectSource Prepare(const source::CanonicalSource&, const ids::ImportMembers&,
                                DirectLimits = {});
    const source::CanonicalSource& canonical() const noexcept;
    const DirectData& data() const noexcept;
    std::size_t owned_payload_bytes() const noexcept;
  private:
    struct Data;
    explicit DirectSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::modelio::solid_control
