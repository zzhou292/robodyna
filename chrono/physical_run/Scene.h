#pragma once
#include "chrono/full_shell/FullShellFrameGeometry.h"
#include "chrono/ReplayFixedCamera.h"
#include "SampleSource.h"
#include <optional>
namespace crash::visual::physical_run {
struct SceneOptions {
    ReplayColorMode colors=ReplayColorMode::PartId;
    ReplayView view=ReplayView::IncidentSide;
    bool wireframe=false;
    std::size_t host_bytes=1024u<<20;
    std::optional<FixedCameraInput> fixed_camera;
};
// Application-owned reader, geometry and transient sample workspace. VSG /
// Vulkan driver buffers are outside this value forecast and measured at render.
struct SceneForecast {
    std::size_t replay_peak_bytes=0,geometry_bytes=0,sample_workspace_bytes=0,peak_host_bytes=0;
};
SceneForecast Forecast(std::size_t replay_peak,std::size_t nodes,std::size_t parents,
    std::size_t triangles,std::size_t points,SceneOptions={});
// Display only: owns fixed visual carriers, never advances Chrono or physics.
// Caller serializes Publish and rendering. Failed seeks retain the visible sample.
class Scene {
  public:
    Scene();
    ~Scene();
    Scene(const Scene&)=delete;
    Scene& operator=(const Scene&)=delete;
    ReplaySceneReport Initialize(const output::physical_run::Replay&,SceneOptions={});
    ReplaySceneReport Initialize(const SampleSource&,SceneOptions={});
    ReplaySceneReport Publish(std::size_t sample);
    chrono::ChSystem& system();
    const ReplayCamera* camera() const noexcept;
    const ReplayStamp* stamp() const noexcept;
    const full_shell::FullShellFrameGeometry* geometry() const noexcept;
    const output::physical_run::Replay* replay() const noexcept; // null for recovered samples
    const SampleSource* samples() const noexcept;
    std::shared_ptr<const chrono::ChVisualShapeTriangleMesh> moving_shape() const noexcept;
    const SceneForecast* forecast() const noexcept;
    double plastic_strain_maximum() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::visual::physical_run
