#pragma once
#include "chrono/AcceptedReplayScene.h"
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
namespace crash::viewer {
class FixedReplayVisual : public chrono::vsg3d::ChVisualSystemVSG {
  public:
    FixedReplayVisual() {m_camera_trackball=false;}
    std::array<std::uint32_t,2> FramebufferSize() const;
};
struct ReplayLighting {double azimuth=0,elevation=0;};
// Chrono captures asset search paths in its visual-system constructor.
std::shared_ptr<FixedReplayVisual> CreateReplayVisual();
ReplayLighting ConfigureReplayVisual(FixedReplayVisual&,chrono::ChSystem&,const visual::ReplayCamera&);
void InitializeReplayVisual(FixedReplayVisual&);
void RenderReplayFrame(FixedReplayVisual&);
void WarmupReplayCapture(FixedReplayVisual&);
output::full_shell::RecordFile CaptureReplayImage(FixedReplayVisual&,const std::filesystem::path&);
} // namespace crash::viewer
