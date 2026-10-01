#pragma once
#include "chrono_vsg/ChVisualSystemVSG.h"
#include <cstddef>
#include <filesystem>
#include <string>

namespace crash::viewer {
struct CapturedImage {
    std::string file, sha256;
    std::size_t bytes = 0;
};
void RenderVsgFrame(chrono::vsg3d::ChVisualSystemVSG&);
// Both renders use unchanged caller-owned geometry. Chrono exports the preceding
// swapchain image. This operation never advances or owns a physical system.
CapturedImage CaptureVsgImage(chrono::vsg3d::ChVisualSystemVSG&, const std::filesystem::path&);
}  // namespace crash::viewer
