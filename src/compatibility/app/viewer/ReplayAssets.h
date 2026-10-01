#pragma once

#include <filesystem>

namespace crash::viewer {
// The frontend supplies an explicit asset directory. Non-Bazel installations
// may supply their declared configured directory as the fallback; no cwd search.
std::filesystem::path ResolveReplayAssets(const std::filesystem::path& requested,
                                         const std::filesystem::path& configured = {});
}  // namespace crash::viewer
