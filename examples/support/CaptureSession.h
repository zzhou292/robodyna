#pragma once

#include "Options.h"
#include "robodyna/simulation/RbSystemFwd.h"
#include "output/ArtifactIO.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include <memory>

namespace robodyna::examples {
using crash::output::Require;

// Fixed camera interaction; demos can still set camera poses/targets explicitly.
class CaptureVisual : public chrono::vsg3d::ChVisualSystemVSG {
  public:
    CaptureVisual() { m_camera_trackball = false; }
};

class CaptureSession {
  public:
    explicit CaptureSession(CaptureOptions);
    ~CaptureSession();
    CaptureSession(const CaptureSession&) = delete;
    CaptureSession& operator=(const CaptureSession&) = delete;

    // Capturing mode only; call before Initialize. Preserves the model's camera,
    // lighting and geometry. Headless systems must have no attached visual.
    void ConfigureVisual(chrono::ChVisualSystem&);
    // Observe initial state0 and each successful original DoStepDynamics call.
    // The real system clock/step counter is checked on every call. At the sample
    // cadence, two renders of the same state capture Chrono's preceding image.
    // Never advances dynamics. Pass nullptr only in explicit headless mode.
    void State(std::uint64_t completed_step, robodyna::simulation::RbSystem&, chrono::ChVisualSystem*);
    // Model-specific finite telemetry is supplied by each original example adapter.
    // Headless completion emits a run summary, never a completed PNG manifest.
    void Finish(robodyna::simulation::RbSystem&, const crash::output::Document& telemetry);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace robodyna::examples
