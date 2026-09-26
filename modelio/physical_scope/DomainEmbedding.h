#pragma once
#include "CanonicalDomain.h"
#include "lib_utils/BoundedStartupArray.h"
#include <memory>

namespace crash::modelio::physical_scope {
struct DomainEmbeddingLimits {
    std::size_t host_bytes = std::size_t{1} << 30;
    std::size_t added_nodes = 1024;
};
struct DomainEmbeddingForecast {
    std::size_t source = 0, original_domain = 0, complete_domain = 0;
    std::size_t object = 0, decode = 0, peak_bytes = 0;
};
// Geometry/source authority only: every original declared node remains the exact
// prefix and the explicitly supplied suffix uses new canonical-external NIDs.
// This does not authorize coefficients, DOFs, constraints, an owner or runtime.
class DomainEmbedding {
  public:
    static DomainEmbeddingForecast Preflight(const PhysicalScope&,
        const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
        tl::util::ConstView<tl::fea::NodalDomainNode> declared_suffix,
        DomainEmbeddingLimits = {});
    static DomainEmbedding Prepare(const PhysicalScope&,
        const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
        tl::util::ConstView<tl::fea::NodalDomainNode> declared_suffix,
        DomainEmbeddingLimits = {});
    const PhysicalScope& source() const noexcept;
    const tl::fea::NodalNodeDomain& original() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    tl::util::ConstView<tl::fea::NodalDomainNode> suffix() const noexcept;
    const DomainEmbeddingForecast& forecast() const noexcept;
    // Increment beyond exact shared source and complete-domain backing. Those
    // two objects are already charged by the existing source contributor caps.
    std::size_t incremental_backing_bytes() const noexcept;
  private:
    struct Data;
    explicit DomainEmbedding(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
namespace detail {
// Shared bounded value checks for qualification. They cannot construct the
// public immutable certificate or bypass complete original-source admission.
void CheckEmbedding(const std::vector<std::uint64_t>& canonical_ids,
    const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
    tl::util::ConstView<tl::fea::NodalDomainNode> declared_suffix);
}
} // namespace crash::modelio::physical_scope
