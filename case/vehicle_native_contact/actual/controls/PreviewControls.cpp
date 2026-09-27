#include "PreviewControls.h"
#include <cmath>
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
PreviewControls ParsePreviewControls(const char* maximum_elapsed_s, const char* stop_file) {
    PreviewControls result;
    if (maximum_elapsed_s) {
        const auto text = BoundedValue(maximum_elapsed_s, "Preview elapsed limit");
        std::size_t used = 0;
        result.maximum_elapsed_s = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(result.maximum_elapsed_s) || result.maximum_elapsed_s < 0)
            throw std::invalid_argument("Preview elapsed limit must be finite and nonnegative");
    }
    if (stop_file) result.stop_file = BoundedValue(stop_file, "Preview stop file");
    return result;
}
PreviewControls ReadPreviewControls() {
    return ParsePreviewControls(std::getenv("ROBO_NATIVE_VEHICLE_MAXIMUM_ELAPSED_S"),
                                std::getenv("ROBO_NATIVE_VEHICLE_STOP_FILE"));
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
