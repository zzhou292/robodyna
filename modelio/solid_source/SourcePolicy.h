#pragma once
#include "VehicleSolidSource.h"

namespace crash::modelio::solid_source::detail {
inline constexpr std::uint64_t AdhesivePart = 2000977;
inline constexpr std::uint64_t FirstRubberPart = 2000477;
inline constexpr std::uint64_t LastRubberPart = 2000484;

struct Census {
    std::size_t parts, parents, solid18, solid24, solid6z;
};
inline bool Supported(Policy policy) noexcept {
    return policy == Policy::OriginalAdhesive18RubberHephS6zV1 ||
           policy == Policy::OriginalAdhesive18ExtendedRubberHephS6zV2;
}
inline Census ExpectedCensus(Policy policy) {
    output::Require(Supported(policy), "Unsupported solid source resolution policy");
    if (policy == Policy::OriginalAdhesive18ExtendedRubberHephS6zV2)
        return {13, 3249, 908, 1991, 350};
    return {9, 2412, 908, 1309, 195};
}
inline bool SelectedRubber(std::uint64_t id, Policy policy) noexcept {
    if (!Supported(policy)) return false;
    if (id >= FirstRubberPart && id <= LastRubberPart) return true;
    if (policy != Policy::OriginalAdhesive18ExtendedRubberHephS6zV2) return false;
    return id == 2000017 || id == 2000393 || id == 2000509 || id == 2000521;
}
inline bool Selected(std::uint64_t id, Policy policy) noexcept {
    return Supported(policy) && (id == AdhesivePart || SelectedRubber(id, policy));
}
} // namespace crash::modelio::solid_source::detail
