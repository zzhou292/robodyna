#include "VsgImageCapture.h"
#include "output/ArtifactIO.h"
#include <vsgXchange/all.h>

namespace crash::viewer {
void RenderVsgFrame(chrono::vsg3d::ChVisualSystemVSG& visual) {
    visual.BeginScene();
    visual.Render();
    visual.EndScene();
}

CapturedImage CaptureVsgImage(chrono::vsg3d::ChVisualSystemVSG& visual, const std::filesystem::path& image) {
    RenderVsgFrame(visual);
    output::Require(visual.Run(), "Window closed before state capture");
    output::Require(!std::filesystem::exists(image), "Refusing screenshot overwrite");
    visual.WriteImageToFile(image.string());
    RenderVsgFrame(visual);
    output::Require(std::filesystem::is_regular_file(image) && std::filesystem::file_size(image) > 0,
                    "VSG did not produce the requested PNG");
    const auto bytes = output::ReadBounded(image, 32u << 20);
    const auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    const auto decoded = vsg::read_cast<vsg::Data>(image.string(), options);
    const auto window = visual.GetWindow();
    output::Require(bool(window), "VSG did not create a capture window");
    const auto size = window->extent2D();
    output::Require(decoded && decoded->width() == size.width && decoded->height() == size.height,
                    "PNG decode or dimensions failed");
    return {image.filename().string(), output::Sha256(bytes), bytes.size()};
}
}  // namespace crash::viewer
