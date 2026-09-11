#pragma once
#include "../Internal.h"
#include <array>

namespace crash::modelio::tied_shell::test {
struct NativeGeometryResult {
    std::array<int, 2> selected{};
    std::array<double, 2> consumed{};
};
NativeGeometryResult NativeGeometry(const std::array<SourceId,4>& master,
    const std::vector<std::array<SourceId,4>>& nodes,
    const std::vector<search_detail::RankedCoefficient>&,
    const std::vector<unsigned>& incidence_order);
} // namespace crash::modelio::tied_shell::test
