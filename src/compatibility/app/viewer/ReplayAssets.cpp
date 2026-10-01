#include "ReplayAssets.h"
#include "output/ArtifactIO.h"

namespace crash::viewer {
std::filesystem::path ResolveReplayAssets(const std::filesystem::path& requested,
                                         const std::filesystem::path& configured) {
    const auto& selected = requested.empty() ? configured : requested;
    output::Require(!selected.empty(), "Replay requires an explicit visualization asset directory (--chrono-data)");
    output::Require(std::filesystem::is_directory(selected), "Replay visualization asset directory is missing");
    const auto directory = std::filesystem::canonical(selected);
    output::Require(std::filesystem::is_regular_file(directory / "logo_robodyna_alpha.png"),
                    "Robodyna visualization data directory is missing its logo");
    output::Require(std::filesystem::is_regular_file(directory / "vsg/fonts/OpenSans-Bold.vsgb"),
                    "Robodyna visualization data directory is missing its VSG font");
    return directory;
}
}  // namespace crash::viewer
