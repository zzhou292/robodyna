#pragma once
#include "Types.h"

namespace crash::output::full_shell::source {
struct NativeParent {
    std::uint32_t canonical_parent = 0, native_family = 0, family_index = 0;
    unsigned native_points = 0;
    PlasticField plastic = PlasticField::Unavailable;
};
struct MappingInput {
    const std::uint32_t* canonical_nodes = nullptr;
    std::size_t node_count = 0;
    const NativeParent* parents = nullptr;
    std::size_t parent_count = 0;
};
// Exact source/runtime ordering and opaque native declarations. This immutable
// mapping owns no nodal state, mass, constraint, force, material history or clock.
class PreparedSourceMapping {
  public:
    static PreparedSourceMapping Prepare(const CanonicalSource&, MappingInput);
    PreparedSourceMapping(const PreparedSourceMapping&) noexcept = default;
    PreparedSourceMapping(PreparedSourceMapping&& other) noexcept : data_(other.data_) {}
    PreparedSourceMapping& operator=(const PreparedSourceMapping&) = delete;
    PreparedSourceMapping& operator=(PreparedSourceMapping&&) = delete;
    const CanonicalSource& source() const noexcept;
    const std::array<NamedArray, 8>& arrays() const noexcept;
    const std::string& digest() const noexcept;
    const std::vector<ParentPoints>& parents() const noexcept;
    std::size_t nodes() const noexcept;
    std::size_t triangles() const noexcept;
    std::size_t payload_bytes() const noexcept;
    // Caller supplies real owner/run/configuration/qualification IDs. Source
    // report and mapping identities are filled from the authenticated mapping.
    // Constructing this context is not authorization to publish accepted frames.
    Context MakeFrameContext(Identity, double fixed_dt, RecordLimits = {}) const;
  private:
    struct Data;
    explicit PreparedSourceMapping(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
// One digest authority for preparation, static records and frame binding. It
// binds the domain tag and ordered explicit layouts/counts/content hashes;
// filesystem paths, run IDs and native C++ object bytes are excluded.
std::string MappingDigest(const std::array<NamedArray, 8>&, arrays::Limits = {});
} // namespace crash::output::full_shell::source
