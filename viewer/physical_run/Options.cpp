#include "Options.h"
#include <cmath>
#include <charconv>
#include <set>
namespace crash::viewer::physical_run {
namespace {
std::array<double, 3> Coordinates(const std::string& text) {
    std::array<double, 3> result;
    std::size_t start = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const auto comma = text.find(',', start);
        output::Require((axis < 2) == (comma != std::string::npos),
            "Camera coordinates require exactly X,Y,Z in metres");
        const auto token = text.substr(start, comma == std::string::npos ? comma : comma-start);
        std::size_t used = 0;
        result[axis] = std::stod(token, &used);
        output::Require(used == token.size() && std::isfinite(result[axis]),
            "Camera coordinates must be finite numbers in metres");
        if (axis < 2) start = comma + 1;
    }
    return result;
}
} // namespace
Options Parse(int argc,char** argv) {
    using output::Require;
    Require(argc>=2,"usage: robo_dyna_physical_replay (RUN_DIR_OR_RECEIPT | --recovered RECOVERED_DESCRIPTOR) [--capture NEW_DIR] [--fps 1..60] [--color part-id|plastic-strain|uniform] [--view incident-side|wall-side | --camera-eye X,Y,Z --camera-target X,Y,Z [--camera-up y|z]] [--wireframe] [--part-palette-seed UINT64] [--require-frames N] [--receipt-sha256 SHA] [--capture-cap-gib 2|6]");
    Options out;
    int first_option=2;
    if (std::string(argv[1])=="--recovered") {
        Require(argc>=3,"--recovered requires an explicit descriptor file");
        out.recovered=true; out.input=argv[2]; first_option=3;
    } else out.input=argv[1];
    visual::FixedCameraInput camera;
    std::set<std::string> seen;
    for(int i=first_option;i<argc;++i) {
        const std::string name=argv[i];
        Require(seen.insert(name).second,"Duplicate physical viewer option");
        if(name=="--wireframe") {out.scene.wireframe=true;continue;}
        Require(i+1<argc,"Missing physical viewer option value");
        const std::string value=argv[++i];
        if(name=="--capture") out.capture=value;
        else if(name=="--fps") {
            std::size_t end=0;out.frames_per_second=std::stod(value,&end);
            Require(end==value.size() && std::isfinite(out.frames_per_second) && out.frames_per_second>=1 &&
                out.frames_per_second<=60,"Recorded sample playback rate must be 1..60");
        } else if(name=="--color") {
            Require(visual::ParseReplayColorMode(value,out.scene.colors),"Unknown physical color mode");
        } else if(name=="--view") {
            Require(visual::ParseReplayView(value,out.scene.view),"Unknown physical camera view");
        } else if (name == "--camera-eye") {
            camera.eye = Coordinates(value);
        } else if (name == "--camera-target") {
            camera.target = Coordinates(value);
        } else if (name == "--camera-up") {
            Require(value == "y" || value == "z", "Camera up axis must be y or z");
            camera.vertical = value == "y" ? visual::ReplayVertical::Y : visual::ReplayVertical::Z;
        } else if(name=="--part-palette-seed") {
            std::uint64_t seed=0;
            const auto result=std::from_chars(value.data(),value.data()+value.size(),seed);
            Require(result.ec==std::errc{} && result.ptr==value.data()+value.size(),
                "Part palette seed requires an unsigned decimal 64-bit integer");
            out.scene.part_palette_seed=seed;
        } else if(name=="--require-frames") {
            std::size_t end=0;const auto n=std::stoull(value,&end);
            Require(end==value.size() && n>0 && n<=10000,"Required sample count must be 1..10000");
            out.require_frames=n;
        } else if(name=="--receipt-sha256") {
            output::arrays::CheckHash(value);out.expected_receipt_sha256=value;
        } else if(name=="--capture-cap-gib") {
            Require(value=="2" || value=="6","Capture cap must be explicit 2 or 6 GiB");
            out.capture_bytes=(value=="2"?2ull:6ull)<<30;
        } else throw std::invalid_argument("Unknown physical viewer option: "+name);
    }
    const bool eye = seen.count("--camera-eye"), target = seen.count("--camera-target");
    Require(eye == target, "Explicit camera requires both --camera-eye and --camera-target");
    Require(!seen.count("--camera-up") || eye, "--camera-up requires the explicit eye/target pair");
    if (eye) {
        Require(!seen.count("--view"), "Explicit camera cannot be combined with --view");
        visual::ReplayCamera validated;
        Require(visual::MakeFixedCamera(camera, validated), "Camera eye/target/up basis is degenerate or unrepresentable");
        out.scene.fixed_camera = camera;
    }
    Require(!seen.count("--part-palette-seed") || out.scene.colors==visual::ReplayColorMode::PartId ||
        out.scene.colors==visual::ReplayColorMode::Automatic,"Part palette seed requires part-id colors");
    return out;
}
} // namespace crash::viewer::physical_run
