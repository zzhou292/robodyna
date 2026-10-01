#include "SessionState.h"
#include "chrono/physics/ChSystem.h"
#include <cmath>
#include <set>

namespace {
void CheckTelemetry(const crash::output::Value& value, unsigned depth,
                    std::size_t& nodes, std::size_t& text_bytes) {
    using crash::output::Require;
    Require(depth <= 8 && ++nodes <= 4096, "Demo telemetry exceeds its bounded summary shape");
    if (value.IsNumber()) Require(std::isfinite(value.GetDouble()), "Nonfinite demo telemetry");
    if (value.IsString()) text_bytes += value.GetStringLength();
    if (value.IsArray()) for (const auto& item : value.GetArray()) CheckTelemetry(item, depth + 1, nodes, text_bytes);
    if (value.IsObject()) {
        std::set<std::string> names;
        for (const auto& item : value.GetObject()) {
            Require(names.emplace(item.name.GetString(), item.name.GetStringLength()).second,
                    "Duplicate demo telemetry field");
            text_bytes += item.name.GetStringLength();
            CheckTelemetry(item.value, depth + 1, nodes, text_bytes);
        }
    }
    Require(text_bytes <= (128u << 10), "Demo telemetry text exceeds metadata reserve");
}
}

namespace robodyna::examples {
void CaptureSession::Impl::CheckEndpoint(chrono::ChSystem& system) const {
    Require(!options.headless || system.GetVisualSystem() == nullptr,
            "Headless demo must not attach a visual system");
    Require(observed && step == options.steps && system.GetNumSteps() == step && system.GetChTime() == time &&
            CaptureTimeMatches(step, time, options.time_step_s), "Demo stopped before its declared physical endpoint");
}
crash::output::Document CaptureSession::Impl::Summary(chrono::ChSystem& system,
                                                    const crash::output::Document& telemetry) const {
    namespace io = crash::output;
    CheckEndpoint(system);
    Require(telemetry.IsObject(), "Demo telemetry must be a model-owned object");
    std::size_t nodes = 0, text_bytes = 0;
    CheckTelemetry(telemetry, 0, nodes, text_bytes);
    io::Document document;
    document.SetObject();
    io::String(document, "schema", "robodyna.chrono_demo_run.v1");
    io::String(document, "source_demo", options.source_demo);
    io::String(document, "source_demo_sha256", options.source_demo_sha256);
    io::String(document, "physics_backend", "chrono_cpu");
    io::Boolean(document, "completed", true);
    io::Boolean(document, "headless", options.headless);
    io::Integer(document, "steps", step);
    io::Number(document, "actual_time_s", time);
    io::Number(document, "time_step_s", options.time_step_s);
    io::Integer(document, "capture_every_steps", options.capture_every);
    io::Integer(document, "frames", frames);
    io::Boolean(document, "physical_restart", false);
    io::Value metrics;
    metrics.CopyFrom(telemetry, document.GetAllocator());
    document.AddMember("telemetry", metrics, document.GetAllocator());
    return document;
}
void CaptureSession::Finish(chrono::ChSystem& system, const crash::output::Document& telemetry) {
    namespace io = crash::output;
    auto& state = *impl_;
    Require(!state.finished, "Demo completion was already published");
    state.CheckEndpoint(system);
    auto summary = state.Summary(system, telemetry);
    if (!state.options.headless) {
        Require(state.frames == CaptureFrameCount(state.options), "Demo did not capture every requested state");
        const auto index = state.index.str();
        Require(index.size() <= (2u << 20), "Demo capture index exceeds metadata reserve");
        io::WriteBytes(state.options.output / "frames.csv", index);
        io::Document metadata;
        metadata.SetObject();
        io::String(metadata, "schema", "robodyna.chrono_live_capture.v1");
        io::String(metadata, "source_demo", state.options.source_demo);
        io::String(metadata, "source_demo_sha256", state.options.source_demo_sha256);
        io::String(metadata, "physics_backend", "chrono_cpu");
        io::String(metadata, "renderer_backend", "vulkan_vsg");
        io::Boolean(metadata, "complete_capture", true);
        io::Boolean(metadata, "all_png_decoded", true);
        io::Boolean(metadata, "simulation_executed_by_viewer", true);
        io::Boolean(metadata, "interpolated_frames", false);
        io::Number(metadata, "deformation_scale", 1);
        io::Integer(metadata, "frames", state.frames);
        io::Integer(metadata, "width", state.width);
        io::Integer(metadata, "height", state.height);
        io::Integer(metadata, "png_bytes", state.png_bytes);
        io::String(metadata, "frame_index_sha256", io::Sha256(index));
        io::Integer(metadata, "final_epoch", state.step);
        io::Number(metadata, "final_time_s", state.time);
        io::Number(metadata, "time_step_s", state.options.time_step_s);
        io::Integer(metadata, "capture_every_steps", state.options.capture_every);
        io::Integer(metadata, "renders_per_sample", 2);
        io::Integer(metadata, "initial_warmup_renders", 1);
        io::Integer(metadata, "loading_workers", state.visual->GetLoadingThreadCount());
        const auto device = state.visual->GetWindow()->getPhysicalDevice()->getProperties();
        io::String(metadata, "vulkan_device_name", device.deviceName);
        io::Integer(metadata, "vulkan_vendor_id", device.vendorID);
        io::Integer(metadata, "vulkan_device_id", device.deviceID);
        io::WriteJson(state.options.output / "run-summary.json", summary);
        io::WriteJson(state.options.output / "manifest.json", metadata);
    } else {
        Require(!state.frames, "Headless demo unexpectedly captured images");
        io::WriteJson(state.options.output / "run-summary.json", summary);
    }
    state.finished = true;
}
}  // namespace robodyna::examples
