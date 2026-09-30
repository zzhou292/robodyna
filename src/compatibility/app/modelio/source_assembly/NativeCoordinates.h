#pragma once
#include "output/full_shell/static_bundle/Types.h"
#include <array>
#include <vector>
namespace crash::modelio::source_nodes {
namespace source = output::full_shell::source;
struct NativeCoordinates {
    std::vector<std::array<double, 3>> positions;
    std::size_t roundtrip_changed_components = 0;
};
// Shared value seam from the authenticated original *NODE path. The caller
// retains CanonicalSource and authenticates the complete source member before
// invoking this routine. It creates no source, physical-domain or clock authority.
// Output follows requested canonical indices, which must be distinct. Original
// values multiplied by the declared length must reproduce canonical SI bits.
NativeCoordinates ReadNativeCoordinates(const source::CanonicalData&,
    const std::vector<std::uint64_t>& canonical_ids,
    const std::vector<std::uint32_t>& requested_canonical_nodes,
    const std::string& original_member);
// Additional peak for this operation: final result + sorted requests + complete
// decoded arrays and the largest decoder temporary. Caller charges borrowed
// canonical/member backing separately; the returned result is included here.
std::size_t NativeCoordinateBytes(const source::CanonicalData&, std::size_t requested_nodes);
} // namespace crash::modelio::source_nodes
