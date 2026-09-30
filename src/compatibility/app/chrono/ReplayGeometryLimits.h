#pragma once

#include <cstddef>

namespace crash::visual {
// Presentation capacity, independent of archive admission and mechanics. These
// limits bound owned geometry/field vectors, not Chrono/VSG's total allocations.
// The caller retains responsibility for the workstation memory guard.
struct ReplayGeometryLimits {
    std::size_t vertices = 4096;
    std::size_t triangles = 8192;
    std::size_t parents = 4096;

    static constexpr ReplayGeometryLimits Vehicle() noexcept {
        return {524288, 1048576, 524288};
    }
    constexpr bool valid() const noexcept {
        const auto maximum = Vehicle();
        return vertices > 0 && vertices <= maximum.vertices &&
               triangles > 0 && triangles <= maximum.triangles &&
               parents > 0 && parents <= maximum.parents;
    }
};
}  // namespace crash::visual
