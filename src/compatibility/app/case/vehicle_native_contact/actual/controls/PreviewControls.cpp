#include "PreviewControls.h"
#include <cmath>
#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <string>
namespace crash::cases::vehicle_native_contact::test {
namespace {
std::string BoundedValue(const char* value, const char* name) {
    const std::string text(value);
    if (text.empty() || text.size() > 4096)
        throw std::invalid_argument(std::string(name) + " must contain 1..4096 characters");
    return text;
}
}
PreviewControls ParsePreviewControls(const char* maximum_elapsed_s, const char* stop_file,
                                    const char* stage_timing, const char* artifact_file_bytes,
                                    const char* capture_qeph_rejection) {
    PreviewControls result;
    if (maximum_elapsed_s) {
        const auto text = BoundedValue(maximum_elapsed_s, "Preview elapsed limit");
        std::size_t used = 0;
        result.maximum_elapsed_s = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(result.maximum_elapsed_s) || result.maximum_elapsed_s < 0)
            throw std::invalid_argument("Preview elapsed limit must be finite and nonnegative");
    }
    if (stop_file) result.stop_file = BoundedValue(stop_file, "Preview stop file");
    if (stage_timing) {
        const auto value = BoundedValue(stage_timing, "Preview stage timing");
        if (value != "0" && value != "1")
            throw std::invalid_argument("Preview stage timing must be exactly 0 or 1");
        result.stage_timing = value == "1";
    }
    if (capture_qeph_rejection) {
        const auto value = BoundedValue(capture_qeph_rejection, "QEPH rejection capture");
        if (value != "0" && value != "1")
            throw std::invalid_argument("QEPH rejection capture must be exactly 0 or 1");
        result.capture_qeph_rejection = value == "1";
    }
    if (artifact_file_bytes) {
        const auto text = BoundedValue(artifact_file_bytes, "Preview artifact byte limit");
        std::size_t bytes = 0;
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), bytes);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
            !bytes)
            throw std::invalid_argument("Preview artifact byte limit must be a positive unsigned decimal");
        result.artifact_file_bytes = bytes;
    }
    return result;
}
PreviewControls ReadPreviewControls() {
    return ParsePreviewControls(std::getenv("ROBO_NATIVE_VEHICLE_MAXIMUM_ELAPSED_S"),
                                std::getenv("ROBO_NATIVE_VEHICLE_STOP_FILE"),
                                std::getenv("ROBO_NATIVE_VEHICLE_STAGE_TIMING"),
                                std::getenv("ROBO_NATIVE_VEHICLE_ARTIFACT_FILE_BYTES"),
                                std::getenv("ROBO_NATIVE_QEPH_REJECTION_CAPTURE"));
}
vehicle_run::Control MakePreviewControl(const PreviewControls& options) {
    // Same option semantics as the existing vehicle-run CLI. The RunLoop checks
    // at accepted boundaries, captures the terminal sample and seals the prefix.
    vehicle_run::Control result;
    result.maximum_elapsed_s = options.maximum_elapsed_s;
    if (!options.stop_file.empty()) {
        result.stop_requested = [path = options.stop_file] { return std::filesystem::exists(path); };
    }
    return result;
}
} // namespace crash::cases::vehicle_native_contact::test
