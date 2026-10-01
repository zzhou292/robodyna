#include "Options.h"
#include "output/ArtifactIO.h"
#include "chrono/core/ChDataPath.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <set>

namespace robodyna::examples {
namespace {
using crash::output::Require;
std::uint64_t Count(const std::string& value) {
    std::uint64_t result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    Require(parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() && result,
            "Demo counts must be positive unsigned integers");
    return result;
}
}

std::size_t CaptureFrameCount(const CaptureOptions& options) {
    Require(options.steps && options.capture_every && options.steps % options.capture_every == 0,
            "Demo steps must be a positive multiple of capture cadence");
    Require(options.steps / options.capture_every < std::numeric_limits<std::size_t>::max(),
            "Demo capture count overflows");
    return static_cast<std::size_t>(options.steps / options.capture_every + 1);
}
void ValidateCaptureOptions(const CaptureOptions& options) {
    Require(options.steps > 0 && options.steps <= 1000000 && std::isfinite(options.time_step_s) &&
            options.time_step_s > 0 && std::isfinite(options.time_step_s * options.steps),
            "Invalid bounded demo horizon");
    const auto count = CaptureFrameCount(options);
    constexpr std::size_t image_cap = 32u << 20, metadata = 4u << 20;
    Require(options.capture_bytes >= metadata && options.capture_bytes <= (std::size_t{6} << 30),
            "Invalid demo capture disk cap");
    Require(options.headless || count <= (options.capture_bytes - metadata) / image_cap,
            "Demo PNG forecast exceeds the declared capture cap");
    Require(!options.output.empty() && !options.chrono_data.empty(), "Demo requires explicit output and asset directories");
}
bool CaptureTimeMatches(std::uint64_t step, double actual, double dt) noexcept {
    if (step > 1000000 || !std::isfinite(actual) || actual < 0 || !std::isfinite(dt) || dt <= 0) return false;
    const double expected = static_cast<double>(step) * dt;
    if (!std::isfinite(expected)) return false;
    const double tolerance = 16 * std::numeric_limits<double>::epsilon() *
        static_cast<double>(step + 1) * std::fmax(1.0, expected);
    return std::fabs(actual - expected) <= tolerance;
}
CaptureOptions ReadCaptureOptions(int argc, char** argv, const std::string& source_demo, double native_dt) {
    Require(argc > 0 && argv && argv[0], "Missing bounded demo executable identity");
    CaptureOptions options;
    options.executable = argv[0]; options.source_demo = source_demo; options.time_step_s = native_dt;
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        Require(seen.insert(key).second, "Duplicate bounded demo option");
        if (key == "--headless") { options.headless = true; continue; }
        Require(key == "--output" || key == "--steps" || key == "--capture-every" || key == "--chrono-data",
                "Supported demo arguments: --output --steps --capture-every --chrono-data [--headless]");
        Require(++i < argc, "Missing bounded demo option value");
        const std::string value = argv[i];
        Require(!value.empty() && value.size() <= 4096, "Invalid bounded demo option value");
        if (key == "--output") options.output = std::filesystem::absolute(value);
        else if (key == "--chrono-data") options.chrono_data = std::filesystem::absolute(value);
        else if (key == "--steps") options.steps = Count(value);
        else options.capture_every = Count(value);
    }
    ValidateCaptureOptions(options);
    Require(std::filesystem::is_directory(options.chrono_data), "Robodyna demo asset directory is missing");
    options.chrono_data = std::filesystem::canonical(options.chrono_data);
    Require(source_demo.rfind("src/compatibility/chrono/src/demos/", 0) == 0 &&
            source_demo.find("..") == std::string::npos, "Demo source identity must be an owned source runfile");
    std::string error;
    using rules_cc::cc::runfiles::Runfiles;
    const std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
    Require(bool(runfiles), "Could not resolve bounded demo runfiles");
    const auto path = runfiles->Rlocation("_main/" + source_demo);
    Require(!path.empty() && std::filesystem::is_regular_file(path), "Original demo source is missing from declared runfiles");
    options.source_demo_sha256 = crash::output::Sha256(crash::output::ReadBounded(path, 2u << 20));
    chrono::SetChronoDataPath(options.chrono_data.string() + '/');
    return options;
}
}  // namespace robodyna::examples
