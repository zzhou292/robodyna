#include "ReplayVsg.h"
#include "output/ArtifactIO.h"
#include <vsgXchange/all.h>
namespace crash::viewer {
output::full_shell::RecordFile CaptureReplayImage(FixedReplayVisual& visual,const std::filesystem::path& image) {
    // Chrono reads the preceding swapchain image. Both renders use the SAME
    // accepted geometry and overlay; no scene publication occurs in between.
    RenderReplayFrame(visual);
    output::Require(visual.Run(),"Window closed before accepted-frame capture");
    output::Require(!std::filesystem::exists(image),"Refusing screenshot overwrite");
    visual.WriteImageToFile(image.string());
    RenderReplayFrame(visual);
    output::Require(std::filesystem::is_regular_file(image) && std::filesystem::file_size(image)>0,
        "VSG did not produce the requested PNG");
    const auto bytes=output::ReadBounded(image,32u<<20);
    const auto options=vsg::Options::create();options->add(vsgXchange::all::create());
    const auto decoded=vsg::read_cast<vsg::Data>(image.string(),options);
    const auto size=visual.FramebufferSize();
    output::Require(decoded && decoded->width()==size[0] && decoded->height()==size[1],"PNG decode or dimensions failed");
    return {image.filename().string(),output::Sha256(bytes),bytes.size()};
}
} // namespace crash::viewer
