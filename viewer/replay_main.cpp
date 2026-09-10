#include "chrono/AcceptedReplayScene.h"
#include "chrono/ReplayParentScalarColors.h"
#include "output/AcceptedReplay.h"
#include "output/ArtifactIO.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <algorithm>

#include "chrono/core/ChDataPath.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
#include <vsg/core/Exception.h>
#include <vsgXchange/all.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace fs = std::filesystem;
using crash::output::Require;
using WallClock = std::chrono::steady_clock;
struct Options {
    fs::path bundle, capture;
    double fps = 10, deformation_scale = 1;
    std::size_t require_frames = 0;
    bool wireframe = false;
    crash::visual::ReplayView view = crash::visual::ReplayView::IncidentSide;
};
Options Parse(int argc, char** argv) {
    Require(argc >= 2, "usage: robo_dyna_replay BUNDLE [--capture NEW_DIR] [--fps 1..60] [--require-frames N] [--wireframe] [--deformation-scale 1..1000] [--view incident-side|wall-side]");
    Options out;
    bool view_supplied = false;
    out.bundle = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--wireframe") out.wireframe = true;
        else {
            Require(i + 1 < argc, "missing replay option value");
            const std::string value = argv[++i];
            if (arg == "--capture") {
                Require(out.capture.empty(), "duplicate capture option");
                out.capture = value;
            } else if (arg == "--fps") {
                std::size_t end = 0;
                out.fps = std::stod(value, &end);
                Require(end == value.size() && std::isfinite(out.fps) && out.fps >= 1 && out.fps <= 60, "playback fps must be 1..60");
            } else if (arg == "--deformation-scale") {
                std::size_t end = 0; out.deformation_scale = std::stod(value, &end);
                Require(end == value.size() && std::isfinite(out.deformation_scale) && out.deformation_scale >= 1 && out.deformation_scale <= 1000, "deformation scale must be 1..1000");
            } else if (arg == "--require-frames") {
                std::size_t end = 0;
                const auto count = std::stoull(value, &end);
                Require(end == value.size() && count > 0 && count <= 1000, "required frame count must be 1..1000");
                out.require_frames = static_cast<std::size_t>(count);
            } else if (arg == "--view") {
                Require(!view_supplied, "duplicate view option");
                Require(crash::visual::ParseReplayView(value, out.view), "view must be incident-side or wall-side");
                view_supplied = true;
            } else throw std::invalid_argument("unknown replay option: " + arg);
        }
    }
    return out;
}
struct Playback {
    bool paused = false, step = false, close = false;
};
void PlasticColorLegend(double maximum) {
    ImGui::TextUnformatted("Color: equivalent plastic strain (max layer per parent)");
    const auto blue=crash::visual::ReplayScalarColor(0);
    const auto yellow=crash::visual::ReplayScalarColor(.5);
    const auto red=crash::visual::ReplayScalarColor(1);
    ImGui::TextColored(ImVec4(blue.R,blue.G,blue.B,1),"0%%");ImGui::SameLine();
    ImGui::TextColored(ImVec4(yellow.R,yellow.G,yellow.B,1),"%.4g%%",50*maximum);ImGui::SameLine();
    ImGui::TextColored(ImVec4(red.R,red.G,red.B,1),"%.4g%%",100*maximum);ImGui::SameLine();
    ImGui::TextUnformatted("| fixed scale for every accepted frame");
}
class ReplayOverlay : public chrono::vsg3d::ChGuiComponentVSG {
  public:
    ReplayOverlay(const crash::output::ReplayInfo& info, const crash::visual::AcceptedReplayScene& scene,
                  Playback& playback, bool capture, double fps)
        : info_(info), scene_(scene), playback_(playback), capture_(capture), fps_(fps) {}
    void render(vsg::CommandBuffer&) override {
        ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_Always);
        const auto flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                           ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;
        if (ImGui::Begin("robo-dyna | accepted replay", nullptr, flags)) {
            ImGui::TextUnformatted(info_.kind == crash::output::ReplayKind::SourceAssemblyWall
                ? "Yaris component assembly | mesh-wall impact | physical scale"
                : info_.kind == crash::output::ReplayKind::SourcePartWall
                ? "Yaris part 2000157 | mesh-wall impact | physical scale"
                : info_.kind == crash::output::ReplayKind::SourcePartElastic
                ? "Yaris part 2000157 | elastic Q4/T3"
                : info_.kind == crash::output::ReplayKind::GuidedPlate
                ? "Guided elastic Q4 plate | physical scale"
                : info_.kind == crash::output::ReplayKind::NormalImpact
                    ? "Nondeforming normal-contact rig" : "Elastic Q4 coupon | physical scale");
            if (info_.kind == crash::output::ReplayKind::SourcePartElastic)
                ImGui::Text("Deformation display: %.1fx | X0 + scale*(X-X0)", scene_.deformation_scale());
            if (scene_.deformation_scale() != 1)
                ImGui::TextUnformatted("Gray wireframe: original undeformed reference");
            ImGui::Text("Fixed view: %s", crash::visual::ReplayViewName(scene_.camera()->view));
            const auto& stamp = *scene_.stamp();
            ImGui::Text("Accepted time: %.6f ms", stamp.time * 1000);
            ImGui::Text("Frame %zu / %zu   Epoch %llu", stamp.index + 1, info_.frame_count,
                        static_cast<unsigned long long>(stamp.epoch));
            ImGui::TextUnformatted((info_.kind == crash::output::ReplayKind::SourcePartWall || info_.kind == crash::output::ReplayKind::SourceAssemblyWall)
                ? (info_.source_plasticity?"Gray wireframe: original mesh wall":"Blue: original elastic part | Gray: placed original mesh wall")
                : info_.kind == crash::output::ReplayKind::SourcePartElastic
                ? "Free part; experimental LAW1; attachments unapplied"
                : info_.kind != crash::output::ReplayKind::ElasticCoupon
                ? "Blue: moving surface   Gray wireframe: canonical wall"
                : "Blue: accepted coupon surface | no wall");
            if(info_.kind==crash::output::ReplayKind::SourcePartWall || info_.kind==crash::output::ReplayKind::SourceAssemblyWall) {
                if(info_.source_plasticity) {
                    ImGui::Text("Initial speed: %.3g m/s | deformation scale: 1x",info_.source_initial_speed_m_per_s);
                    PlasticColorLegend(info_.plastic_strain_color_max);
                    if(info_.kind==crash::output::ReplayKind::SourceAssemblyWall && info_.source_assembly) {
                        const auto& assembly=*info_.source_assembly;
                        ImGui::Text("%zu source parts | %zu shells | %zu internal rigid groups",
                                    assembly.part_ids.size(),assembly.parents,assembly.groups);
                        ImGui::TextUnformatted("External connections released");
                    } else ImGui::TextUnformatted("Isolated source part; vehicle attachments not included");
                } else ImGui::TextUnformatted("Experimental LAW1; source attachments unapplied");
                if(!info_.horizon_complete)ImGui::TextUnformatted("Accepted prefix only | requested horizon stopped early");
            }
            if (capture_) ImGui::TextUnformatted("Indexed PNG capture | fixed camera");
            else {
                ImGui::Text("Playback: %.1f recorded frames/s", fps_);
                if (ImGui::Button(playback_.paused ? "Play" : "Pause")) playback_.paused = !playback_.paused;
                ImGui::SameLine();
                if (ImGui::Button("Next frame")) { playback_.step = true; playback_.paused = true; }
                ImGui::SameLine();
                if (ImGui::Button("Close")) playback_.close = true;
                if (stamp.index + 1 == info_.frame_count) ImGui::TextUnformatted("End of accepted frames");
            }
        }
        ImGui::End();
    }
  private:
    const crash::output::ReplayInfo& info_;
    const crash::visual::AcceptedReplayScene& scene_;
    Playback& playback_;
    bool capture_;
    double fps_;
};
// Use the same protected extension as ChVehicleVisualSystemVSG: no trackball
// handler is installed, so user events cannot change the reference camera.
class FixedReplayVisual : public chrono::vsg3d::ChVisualSystemVSG {
  public:
    FixedReplayVisual() { m_camera_trackball = false; }
    std::array<std::uint32_t, 2> FramebufferSize() const {
        if (!m_window) throw std::logic_error("VSG did not create a window");
        const auto extent = m_window->extent2D();
        return {extent.width, extent.height};
    }
};
chrono::ChVector3d Vector(const std::array<double, 3>& value) { return {value[0], value[1], value[2]}; }
void Array(crash::output::Document& document, const char* name, const std::array<double, 3>& values) {
    rapidjson::Value array(rapidjson::kArrayType);
    for (double value : values) array.PushBack(value, document.GetAllocator());
    document.AddMember(rapidjson::Value(name, document.GetAllocator()), array, document.GetAllocator());
}
bool Within(const fs::path& child, const fs::path& parent) {
    auto a = child.begin(), b = parent.begin();
    while (b != parent.end()) {
        if (a == child.end() || *a != *b) return false;
        ++a; ++b;
    }
    return true;
}
int Failed(const fs::path& capture_directory, const std::string& message) {
    if (!capture_directory.empty()) {
        try { crash::output::WriteBytes(capture_directory / "failure.txt", message + '\n'); } catch (...) {}
    }
    std::cerr << "robo-dyna replay: " << message << '\n';
    return 1;
}
}  // namespace

int main(int argc, char** argv) {
    fs::path capture_directory;
    try {
        const auto options = Parse(argc, argv);
        crash::output::AcceptedReplay reader;
        auto report = reader.Open(options.bundle);
        Require(report.status == crash::output::ReplayStatus::Ok, report.diagnostic.c_str());
        auto info = *reader.info();
        Require(options.deformation_scale == 1 || info.kind == crash::output::ReplayKind::SourcePartElastic, "Deformation scaling is explicit source-part presentation only");
        if (options.deformation_scale != 1) {
            const auto reference = reader.frame()->mesh->GetCoordsVertices();
            info.bounds_min.fill(std::numeric_limits<double>::infinity());
            info.bounds_max.fill(-std::numeric_limits<double>::infinity());
            for (std::size_t i = 0; i < info.frame_count; ++i) {
                report = reader.Load(i); Require(report.status == crash::output::ReplayStatus::Ok, report.diagnostic.c_str());
                const auto& vertices = reader.frame()->mesh->GetCoordsVertices();
                for (std::size_t n = 0; n < reference.size(); ++n) for (unsigned axis = 0; axis < 3; ++axis) {
                    const double x = reference[n][axis] + options.deformation_scale * (vertices[n][axis] - reference[n][axis]);
                    Require(std::isfinite(x), "Magnified display bounds are nonfinite");
                    info.bounds_min[axis] = std::min(info.bounds_min[axis], x);
                    info.bounds_max[axis] = std::max(info.bounds_max[axis], x);
                }
            }
            report = reader.Load(0); Require(report.status == crash::output::ReplayStatus::Ok, report.diagnostic.c_str());
        }
        Require(!options.require_frames || options.require_frames == info.frame_count, "accepted frame count differs from required count");
        crash::visual::AcceptedReplayScene scene;
        const auto initialized = scene.Initialize(info, *reader.frame(), reader.wall(), options.wireframe,
                                                  options.deformation_scale, options.view);
        Require(initialized.status == crash::visual::ReplaySceneStatus::Ok, initialized.message);
        const bool capture = !options.capture.empty();
        if (capture) {
            const auto directory = fs::weakly_canonical(options.capture);
            Require(!Within(directory, fs::weakly_canonical(options.bundle)), "capture directory must be outside the immutable input bundle");
            Require(!fs::exists(directory) && fs::is_directory(directory.parent_path()), "capture directory must be new with an existing parent");
            Require(fs::create_directory(directory), "cannot create capture directory");
            capture_directory = directory;
        }
#ifdef ROBO_DYNA_CHRONO_DATA_DIR
        chrono::SetChronoDataPath(ROBO_DYNA_CHRONO_DATA_DIR);
#endif
        Require(fs::is_regular_file(chrono::GetChronoDataFile("logo_chrono_alpha.png")), "Chrono visualization data directory is missing its logo");
        Playback playback;
        auto visual = std::make_shared<FixedReplayVisual>();
        visual->AttachSystem(&scene.system());
        visual->SetLoadingThreadCount(1);
        visual->SetTargetRenderFPS(0);  // Never skip a requested capture.
        visual->SetWindowSize(1280, 720);
        visual->SetWindowPosition(60, 60);
        visual->SetWindowTitle("robo-dyna | accepted simulation replay");
        visual->SetBackgroundColor(chrono::ChColor(0.06f, 0.08f, 0.11f));
        visual->SetCameraVertical(scene.camera()->vertical == crash::visual::ReplayVertical::Y
            ? chrono::CameraVerticalDir::Y : chrono::CameraVerticalDir::Z);
        visual->AddCamera(Vector(scene.camera()->position), Vector(scene.camera()->target));
        visual->SetCameraAngleDeg(scene.camera()->vertical_fov_degrees);
        // Illuminate from the fixed camera side. The owning API clamps azimuth
        // to [0, 2*pi], so a negative angle would silently light the other side.
        const auto& camera = *scene.camera();
        const double dx = camera.position[0] - camera.target[0];
        const double dy = camera.position[1] - camera.target[1];
        const double dz = camera.position[2] - camera.target[2];
        // Owning VSG uses (x,z) azimuth with reversed z for Y-up.
        const bool y_up = camera.vertical == crash::visual::ReplayVertical::Y;
        double light_azimuth = y_up ? std::atan2(-dz, dx) : std::atan2(dy, dx);
        if (light_azimuth < 0) light_azimuth += 2 * std::acos(-1.0);
        const double light_elevation = y_up ? std::atan2(dy, std::hypot(dx, dz))
                                           : std::atan2(dz, std::hypot(dx, dy));
        visual->SetLightIntensity(1.0f);
        visual->SetLightDirection(light_azimuth, light_elevation);
        visual->SetBaseGuiVisibility(false);
        visual->AddGuiComponent(std::make_shared<ReplayOverlay>(info, scene, playback, capture, options.fps));
        visual->Initialize();  // Binds the scene exactly once.
        Require(visual->IsInitialized(), "VSG initialization did not complete");
        ImGui::GetIO().IniFilename = nullptr;  // Playback never writes Chrono's shared imgui.ini.
        Require(visual->GetLoadingThreadCount() == 1, "VSG loading-worker budget changed");
        const auto dimensions = visual->FramebufferSize();
        const auto device_properties = visual->GetWindow()->getPhysicalDevice()->getProperties();
        Require(dimensions[0] > 0 && dimensions[0] <= 2560 && dimensions[1] > 0 && dimensions[1] <= 1440,
                "VSG framebuffer exceeds bounded replay dimensions");
        // Test actual post-initialize rejection in every runtime smoke.
        bool locked = false;
        try { visual->SetLoadingThreadCount(2); } catch (const std::logic_error&) { locked = true; }
        Require(locked && visual->GetLoadingThreadCount() == 1, "VSG loading-worker configuration did not freeze");
        auto png_options = vsg::Options::create();
        png_options->add(vsgXchange::all::create());
        std::ostringstream index;
        index << "frame,epoch,accepted_time_s,file,bytes,sha256\n" << std::setprecision(17);
        std::size_t rendered = 0;
        bool first = true;
        auto next_frame_at = WallClock::now();
        if (capture) {
            // ImGui deliberately hides a newly created auto-sized window for
            // its first frame while measuring it. Prime that layout at row 0;
            // the paired renders below then capture a visible initial overlay.
            visual->BeginScene();
            visual->Render();
            visual->EndScene();
            Require(visual->Run(), "window closed during initial overlay warmup");
        }
        while (visual->Run()) {
            const auto now = WallClock::now();
            const bool advance = !first && (capture || playback.step || (!playback.paused && now >= next_frame_at));
            if (advance && scene.stamp()->index + 1 < info.frame_count) {
                report = reader.Load(scene.stamp()->index + 1);
                Require(report.status == crash::output::ReplayStatus::Ok, report.diagnostic.c_str());
                const auto published = scene.Publish(*reader.frame());
                Require(published.status == crash::visual::ReplaySceneStatus::Ok, published.message);
                playback.step = false;
                next_frame_at = now + std::chrono::duration_cast<WallClock::duration>(std::chrono::duration<double>(1 / options.fps));
            }
            fs::path image;
            if (capture) {
                // Chrono ExportScreenImage reads imageIndex(1), the previous
                // rendered frame. Populate that image with this accepted state
                // before requesting capture; do not advance geometry or overlay
                // between these two renders (including initial frame zero).
                visual->BeginScene();
                visual->Render();
                visual->EndScene();
                Require(visual->Run(), "window closed before the accepted-frame capture");
                std::ostringstream name;
                name << "frame-" << std::setw(6) << std::setfill('0') << scene.stamp()->index << ".png";
                image = capture_directory / name.str();
                Require(!fs::exists(image), "refusing screenshot overwrite");
                visual->WriteImageToFile(image.string());  // Request before Render.
            }
            visual->BeginScene();
            visual->Render();  // Uses changed vertices and actual recomputed face normals.
            visual->EndScene();
            if (capture) {
                Require(fs::is_regular_file(image) && fs::file_size(image) > 0, "VSG did not produce the requested PNG");
                const auto bytes = crash::output::ReadBounded(image, 32 * 1024 * 1024);
                auto decoded = vsg::read_cast<vsg::Data>(image.string(), png_options);
                Require(decoded && decoded->width() == dimensions[0] && decoded->height() == dimensions[1], "PNG decode or dimensions failed");
                index << scene.stamp()->index << ',' << scene.stamp()->epoch << ',' << scene.stamp()->time << ','
                      << image.filename().string() << ',' << bytes.size() << ',' << crash::output::Sha256(bytes) << '\n';
                ++rendered;
                if (rendered == info.frame_count) break;
            }
            if (first) {
                next_frame_at = now + std::chrono::duration_cast<WallClock::duration>(std::chrono::duration<double>(1 / options.fps));
                first = false;
            }
            if (playback.close) { visual->Quit(); break; }
            if (!capture) std::this_thread::sleep_for(std::chrono::milliseconds(33));  // Bounded interactive redraw cadence.
        }
        if (capture) {
            Require(rendered == info.frame_count && scene.stamp()->epoch == info.final_epoch && scene.stamp()->time == info.final_time,
                    "window closed or capture ended before the complete accepted horizon");
            crash::output::WriteBytes(capture_directory / "frames.csv", index.str());
            crash::output::Document manifest;
            manifest.SetObject();
            crash::output::String(manifest, "schema", "robo-dyna.accepted-replay-capture.v1");
            crash::output::String(manifest, "input_bundle", fs::weakly_canonical(options.bundle).string());
            crash::output::String(manifest, "input_schema", info.schema);
            crash::output::String(manifest, "input_scope", info.scope);
            if(info.source_plasticity) {
                if(info.source_assembly) {
                    const auto& a=*info.source_assembly;crash::output::Document source;source.SetObject();
                    crash::output::Integer(source,"source_instance_id",a.source_instance_id);
                    crash::output::String(source,"inventory_sha256",a.inventory_sha256);
                    crash::output::Integer(source,"inventory_bytes",a.inventory_bytes);
                    crash::output::String(source,"boundary_policy",a.boundary_policy);
                    crash::output::Integer(source,"parent_count",a.parents);crash::output::Integer(source,"qeph_count",a.qeph);
                    crash::output::Integer(source,"t3_count",a.t3);crash::output::Integer(source,"group_count",a.groups);
                    crash::output::Integer(source,"member_count",a.members);
                    const auto ids=[&](const char* key,const auto& list) {
                        crash::output::Value values(rapidjson::kArrayType);
                        for(auto id:list)values.PushBack(crash::output::Value().SetUint64(id),source.GetAllocator());
                        source.AddMember(crash::output::Value(key,source.GetAllocator()),values,source.GetAllocator());
                    };
                    ids("part_ids",a.part_ids);ids("material_ids",a.material_ids);ids("section_ids",a.section_ids);ids("curve_ids",a.curve_ids);
                    crash::output::Value value;value.CopyFrom(source,manifest.GetAllocator());
                    manifest.AddMember("source_assembly",value,manifest.GetAllocator());
                } else {
                    crash::output::String(manifest,"material_model",info.material_model);
                    crash::output::String(manifest,"material_policy",info.material_policy);
                }
                crash::output::String(manifest,"surface_color_quantity","maximum accepted layer equivalent plastic strain per original source parent");
                crash::output::String(manifest,"surface_color_palette","piecewise linear blue(0.12,0.64,0.94), yellow(0.98,0.84,0.16), red(0.90,0.12,0.10)");
                crash::output::String(manifest,"surface_color_scale_policy","fixed 0..max(0.001, accepted final maximum plastic strain); endpoint saturation; no frame autoscale");
                crash::output::Number(manifest,"surface_color_min",0);
                crash::output::Number(manifest,"surface_color_max",info.plastic_strain_color_max);
                crash::output::String(manifest,"surface_color_units","dimensionless; visible legend in percent");
                crash::output::Boolean(manifest,"surface_color_flat_per_source_parent",true);
                crash::output::Number(manifest,"source_initial_speed_m_per_s",info.source_initial_speed_m_per_s);
            }
            crash::output::Integer(manifest, "owner_id", info.owner_id);
            crash::output::Integer(manifest, "frame_count", rendered);
            crash::output::Integer(manifest, "renders_per_accepted_frame", 2);
            crash::output::Integer(manifest, "initial_uncaptured_warmup_renders", 1);
            crash::output::String(manifest, "capture_source", "previous_rendered_frame_of_same_accepted_state");
            crash::output::Number(manifest, "light_azimuth", light_azimuth);
            crash::output::Number(manifest, "light_elevation", light_elevation);
            crash::output::String(manifest, "vulkan_device_name", device_properties.deviceName);
            crash::output::Integer(manifest, "vulkan_vendor_id", device_properties.vendorID);
            crash::output::Integer(manifest, "vulkan_device_id", device_properties.deviceID);
            crash::output::Integer(manifest, "final_epoch", scene.stamp()->epoch);
            crash::output::Number(manifest, "final_time", scene.stamp()->time);
            crash::output::Integer(manifest, "width", dimensions[0]);
            crash::output::Integer(manifest, "height", dimensions[1]);
            crash::output::Number(manifest, "presentation_frame_rate_fps", options.fps);
            crash::output::Integer(manifest, "loading_workers", visual->GetLoadingThreadCount());
            crash::output::Boolean(manifest, "complete", true);
            crash::output::Boolean(manifest, "all_png_decoded", true);
            crash::output::Boolean(manifest, "simulation_executed_by_viewer", false);
            crash::output::Boolean(manifest, "deformation_scaled", options.deformation_scale != 1);
            crash::output::Number(manifest, "deformation_scale", options.deformation_scale);
            crash::output::Boolean(manifest, "original_reference_outline", options.deformation_scale != 1);
            crash::output::String(manifest, "deformation_display_law", "X0 + scale*(accepted_X-X0)");
            crash::output::Boolean(manifest, "wireframe", options.wireframe);
            crash::output::String(manifest, "wall_display", reader.wall() ? "gray_wireframe" : "none");
            if(info.kind==crash::output::ReplayKind::SourcePartWall || info.kind==crash::output::ReplayKind::SourceAssemblyWall) {
                crash::output::Boolean(manifest,"input_horizon_complete",info.horizon_complete);
                crash::output::String(manifest,"input_stop_reason",info.stop_reason);
                crash::output::String(manifest,"wall_geometry","actual placed original canonical mesh; original source and X transform archived separately");
            }
            crash::output::String(manifest, "camera_view", crash::visual::ReplayViewName(scene.camera()->view));
            Array(manifest, "camera_position", scene.camera()->position);
            Array(manifest, "camera_target", scene.camera()->target);
            crash::output::String(manifest, "camera_vertical", scene.camera()->vertical == crash::visual::ReplayVertical::Y ? "Y" : "Z");
            crash::output::Number(manifest, "camera_vertical_fov_degrees", scene.camera()->vertical_fov_degrees);
            crash::output::String(manifest, "frame_index_sha256", crash::output::Sha256(index.str()));
            crash::output::WriteJson(capture_directory / "manifest.json", manifest);  // Completion marker is last.
            std::cout << "Captured " << rendered << " accepted frames to " << capture_directory << '\n';
        }
        return 0;
    } catch (const vsg::Exception& error) {
        // VSG's pinned Exception is not derived from std::exception.
        return Failed(capture_directory, error.message + " (VSG result " + std::to_string(error.result) + ")");
    } catch (const std::exception& error) {
        return Failed(capture_directory, error.what());
    } catch (...) {
        return Failed(capture_directory, "unknown replay failure");
    }
}
