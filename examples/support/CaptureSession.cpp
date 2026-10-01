#include "SessionState.h"
#include "viewer/VsgImageCapture.h"
#include "chrono/physics/ChSystem.h"
#include <iomanip>
#include <locale>

namespace robodyna::examples {
CaptureSession::CaptureSession(CaptureOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {
    auto& state = *impl_;
    ValidateCaptureOptions(state.options);
    const auto path = std::filesystem::weakly_canonical(state.options.output);
    const auto assets = std::filesystem::weakly_canonical(state.options.chrono_data);
    const auto relative = path.lexically_relative(assets);
    Require(relative.empty() || *relative.begin() == "..", "Demo output must be outside immutable assets");
    Require(!std::filesystem::exists(path) && !std::filesystem::is_symlink(path) &&
            std::filesystem::is_directory(path.parent_path()), "Demo output must be new with an existing parent");
    Require(!state.options.source_demo.empty() && state.options.source_demo.size() <= 4096 &&
            state.options.source_demo_sha256.size() == 64 &&
            state.options.source_demo_sha256.find_first_not_of("0123456789abcdef") == std::string::npos,
            "Demo source has no authenticated digest");
    Require(std::filesystem::create_directory(path), "Could not create bounded demo output");
    state.options.output = path;
    state.index.imbue(std::locale::classic());
    state.index << "sample,epoch,accepted_time_s,file,bytes,sha256\n" << std::setprecision(17);
}
CaptureSession::~CaptureSession() = default;

void CaptureSession::ConfigureVisual(chrono::ChVisualSystem& base) {
    auto& state = *impl_;
    Require(!state.options.headless && !state.observed && !state.configured, "Invalid demo visual configuration phase");
    auto* visual = dynamic_cast<CaptureVisual*>(&base);
    Require(visual && !visual->IsInitialized(), "Bounded demos require an uninitialized CaptureVisual");
    visual->SetLoadingThreadCount(1);
    visual->SetTargetRenderFPS(0);
    visual->SetBaseGuiVisibility(false);
    state.visual = visual;
    state.configured = true;
}
void CaptureSession::State(std::uint64_t step, chrono::ChSystem& system, chrono::ChVisualSystem* base) {
    auto& state = *impl_;
    Require(!state.finished && step <= state.options.steps &&
            ((!state.observed && step == 0) || (state.observed && step == state.step + 1)),
            "Demo state observations must include every successful original step exactly once");
    // Chrono can initialize an attached visual from its first OnSetup callback.
    // Reject attachment at state 0, before any native dynamics step can do that.
    if (state.options.headless)
        Require(!base && !state.visual && system.GetVisualSystem() == nullptr,
                "Headless demo must not attach a visual system");
    const auto time = system.GetChTime();
    Require(system.GetNumSteps() == step && CaptureTimeMatches(step, time, state.options.time_step_s),
            "Demo physical clock or completed-step count differs from its native horizon");
    state.step = step; state.time = time; state.observed = true;
    if (state.options.headless) return;
    Require(state.configured && base == state.visual && state.visual->IsInitialized(), "Demo renderer identity or initialization differs");
    if (step % state.options.capture_every) return;
    auto& visual = *state.visual;
    Require(visual.Run() && visual.GetLoadingThreadCount() == 1, "Demo capture window closed or worker budget changed");
    const auto window = visual.GetWindow();
    Require(bool(window), "Demo VSG window is unavailable");
    const auto size = window->extent2D();
    Require(size.width && size.height && !(size.width % 2) && !(size.height % 2) &&
            size.width <= 2560 && size.height <= 1440, "Demo framebuffer exceeds its bounded even dimensions");
    if (!state.frames) {
        state.width = size.width; state.height = size.height;
        ImGui::GetIO().IniFilename = nullptr;
        crash::viewer::RenderVsgFrame(visual);
    }
    Require(state.width == size.width && state.height == size.height, "Demo framebuffer changed during capture");
    Require(state.png_bytes <= state.options.capture_bytes - (4u << 20) &&
            (32u << 20) <= state.options.capture_bytes - (4u << 20) - state.png_bytes,
            "Demo PNG byte cap exhausted");
    std::ostringstream name;
    name << "frame-" << std::setw(6) << std::setfill('0') << state.frames << ".png";
    const auto image = crash::viewer::CaptureVsgImage(visual, state.options.output / name.str());
    Require(system.GetChTime() == time && system.GetNumSteps() == step, "Rendering advanced the demo's physical system");
    const auto after = visual.GetWindow()->extent2D();
    Require(after.width == state.width && after.height == state.height, "Framebuffer changed while capturing one state");
    state.png_bytes += image.bytes;
    state.index << state.frames << ',' << step << ',' << time << ',' << image.file << ',' << image.bytes << ',' << image.sha256 << '\n';
    ++state.frames;
}
}  // namespace robodyna::examples
