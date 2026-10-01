#include "ReplayVsg.h"
#include "VsgImageCapture.h"
namespace crash::viewer {
output::full_shell::RecordFile CaptureReplayImage(FixedReplayVisual& visual,const std::filesystem::path& image) {
    const auto file = CaptureVsgImage(visual, image);
    return {file.file, file.sha256, file.bytes};
}
} // namespace crash::viewer
