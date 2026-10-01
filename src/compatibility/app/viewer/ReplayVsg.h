#pragma once
#include "robodyna/simulation/RbSystemFwd.h"
#include "chrono/AcceptedReplayScene.h"
#include "chrono/ReplayClipping.h"
#include <optional>
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
namespace crash::viewer {
class FixedReplayVisual : public chrono::vsg3d::ChVisualSystemVSG {
  public:
    FixedReplayVisual() {m_camera_trackball=false;}
    std::array<std::uint32_t,2> FramebufferSize() const;
    // Call after Initialize and before the first render. Uses the actual VSG
    // camera pose and changes only the perspective near/far distances.
    void ConfigureClipping(const visual::ReplayBounds&);
    const visual::ReplayClipping& Clipping() const;
  private:
    std::optional<visual::ReplayClipping> clipping_;
};
struct ReplayLighting {double azimuth=0,elevation=0;};
// Chrono captures asset search paths in its visual-system constructor.
std::shared_ptr<FixedReplayVisual> CreateReplayVisual(const std::filesystem::path& asset_directory = {});
ReplayLighting ConfigureReplayVisual(FixedReplayVisual&,robodyna::simulation::RbSystem&,const visual::ReplayCamera&);
void InitializeReplayVisual(FixedReplayVisual&);
void RenderReplayFrame(FixedReplayVisual&);
void WarmupReplayCapture(FixedReplayVisual&);
output::full_shell::RecordFile CaptureReplayImage(FixedReplayVisual&,const std::filesystem::path&);
} // namespace crash::viewer
