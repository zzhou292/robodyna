#include "ReplayVsg.h"
#include "output/ArtifactIO.h"
#include "chrono/core/ChDataPath.h"
#include <cmath>
#include <stdexcept>
namespace crash::viewer {
std::array<std::uint32_t,2> FixedReplayVisual::FramebufferSize() const {
    if(!m_window) throw std::logic_error("VSG did not create a window");
    const auto extent=m_window->extent2D();
    return {extent.width,extent.height};
}
std::shared_ptr<FixedReplayVisual> CreateReplayVisual() {
#ifdef ROBO_DYNA_CHRONO_DATA_DIR
    chrono::SetChronoDataPath(ROBO_DYNA_CHRONO_DATA_DIR);
#endif
    output::Require(std::filesystem::is_regular_file(chrono::GetChronoDataFile("logo_chrono_alpha.png")),
        "Chrono visualization data directory is missing its logo");
    output::Require(std::filesystem::is_regular_file(chrono::GetChronoDataFile("vsg/fonts/OpenSans-Bold.vsgb")),
        "Chrono visualization data directory is missing its VSG font");
    return std::make_shared<FixedReplayVisual>();
}
ReplayLighting ConfigureReplayVisual(FixedReplayVisual& visual,chrono::ChSystem& system,
        const visual::ReplayCamera& camera) {
    visual.AttachSystem(&system);
    visual.SetLoadingThreadCount(1);
    visual.SetTargetRenderFPS(0);
    visual.SetWindowSize(1280,720);
    visual.SetWindowPosition(60,60);
    visual.SetWindowTitle("robo-dyna | accepted simulation replay");
    visual.SetBackgroundColor(chrono::ChColor(.06f,.08f,.11f));
    const bool y_up=camera.vertical==visual::ReplayVertical::Y;
    visual.SetCameraVertical(y_up?chrono::CameraVerticalDir::Y:chrono::CameraVerticalDir::Z);
    const auto vector=[](const auto& x) {return chrono::ChVector3d{x[0],x[1],x[2]};};
    visual.AddCamera(vector(camera.position),vector(camera.target));
    visual.SetCameraAngleDeg(camera.vertical_fov_degrees);
    const double dx=camera.position[0]-camera.target[0];
    const double dy=camera.position[1]-camera.target[1];
    const double dz=camera.position[2]-camera.target[2];
    ReplayLighting light;
    light.azimuth=y_up?std::atan2(-dz,dx):std::atan2(dy,dx);
    if(light.azimuth<0) light.azimuth+=2*std::acos(-1.);
    light.elevation=y_up?std::atan2(dy,std::hypot(dx,dz)):std::atan2(dz,std::hypot(dx,dy));
    visual.SetLightIntensity(1.f);
    visual.SetLightDirection(light.azimuth,light.elevation);
    visual.SetBaseGuiVisibility(false);
    return light;
}
void InitializeReplayVisual(FixedReplayVisual& visual) {
    visual.Initialize();
    output::Require(visual.IsInitialized(),"VSG initialization did not complete");
    ImGui::GetIO().IniFilename=nullptr;
    output::Require(visual.GetLoadingThreadCount()==1,"VSG loading-worker budget changed");
    const auto size=visual.FramebufferSize();
    output::Require(size[0]>0 && size[0]<=2560 && size[1]>0 && size[1]<=1440,
        "VSG framebuffer exceeds bounded replay dimensions");
    bool locked=false;
    try {visual.SetLoadingThreadCount(2);} catch(const std::logic_error&) {locked=true;}
    output::Require(locked && visual.GetLoadingThreadCount()==1,"VSG loading-worker configuration did not freeze");
}
void RenderReplayFrame(FixedReplayVisual& visual) {
    visual.BeginScene();visual.Render();visual.EndScene();
}
void WarmupReplayCapture(FixedReplayVisual& visual) {
    RenderReplayFrame(visual);
    output::Require(visual.Run(),"Window closed during initial overlay warmup");
}
} // namespace crash::viewer
