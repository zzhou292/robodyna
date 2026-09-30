#include "Values.h"
#include "output/ArtifactIO.h"
#include "lib_src/elements/solid_common/BrickFrame.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::coated {
std::array<std::uint32_t, 8> ReaderSlots(ReaderKind kind,
        const std::array<std::uint32_t, 8>& declared, const std::vector<Node>& nodes) {
    output::Require(kind == ReaderKind::Hex8 || kind == ReaderKind::DeclaredPenta6,
                    "Unknown V5 native solid reader declaration");
    const unsigned count = kind == ReaderKind::Hex8 ? 8 : 6;
    for (unsigned i = 0; i < count; ++i) {
        output::Require(declared[i] < nodes.size(), "Native solid reader node is outside physical domain");
        const auto value = nodes[declared[i]].native_position;
        output::Require(std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z),
                        "Native solid reader coordinate is nonfinite");
        if (kind == ReaderKind::DeclaredPenta6)
            for (unsigned j = 0; j < i; ++j)
                output::Require(declared[i] != declared[j], "Declared PENTA6 requires six distinct source nodes");
    }
    std::array<std::uint32_t, 8> result{};
    if (kind == ReaderKind::DeclaredPenta6) {
        // HM_READ_SOLID's actual /PENTA6 arm, before INITIA/S6ZCOOR3.
        // In particular, do not rotate active slots using a mechanics reference.
        constexpr unsigned slots[]{0, 1, 2, 0, 3, 4, 5, 3};
        for (unsigned i = 0; i < 8; ++i) result[i] = declared[slots[i]];
    } else {
        tl::math::Vec3 positions[8];
        for (unsigned i = 0; i < 8; ++i) positions[i] = nodes[declared[i]].native_position;
        // Existing qualified CHECKVOLUME_8N operation sequence, in the same
        // original native coordinate space as the reader, not an SI roundtrip.
        const double volume = tl::fea::solid_common::SignedCenterVolume(positions);
        output::Require(std::isfinite(volume) && volume != 0, "Native H8 reader volume is nonfinite or zero");
        for (unsigned i = 0; i < 8; ++i) result[i] = declared[volume < 0 ? (i + 4) % 8 : i];
    }
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
