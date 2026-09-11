#include "TiedShellSearchGeometry.h"
#include "search_geometry/Internal.h"
#include <algorithm>

namespace crash::modelio::tied_shell {
struct TiedShellSearchGeometry::Storage {
    TiedShellPacking packing;
    SearchGeometryData data;
    Storage(const TiedShellPacking& input, SearchGeometryData value)
        : packing(input), data(std::move(value)) {}
};
std::size_t TiedShellSearchGeometry::Forecast(const TiedShellPacking& input, SearchGeometryLimits limits) {
    return search_detail::Preflight(input.declaration().canonical().data(),
        input.declaration().data(), input.data(), limits);
}
TiedShellSearchGeometry TiedShellSearchGeometry::Prepare(const TiedShellPacking& input,
        const std::string& member, SearchGeometryLimits limits) {
    auto value = search_detail::Build(input.declaration().canonical().data(),
        input.declaration().data(), input.data(), member, limits);
    return TiedShellSearchGeometry(std::make_shared<const Storage>(input, std::move(value)));
}
const TiedShellPacking& TiedShellSearchGeometry::packing() const noexcept { return storage_->packing; }
const SearchGeometryData& TiedShellSearchGeometry::data() const noexcept { return storage_->data; }
bool TiedShellSearchGeometry::PhysicalOwnNode(std::size_t secondary, std::size_t master) const {
    const auto node = data().secondary_working_nodes.at(secondary);
    const auto& nodes = data().masters.at(master).working_nodes;
    return std::find(nodes.begin(), nodes.end(), node) != nodes.end();
}
} // namespace crash::modelio::tied_shell
