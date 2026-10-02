#include "Resources.h"
#include "chrono/core/ChDataPath.h"
#include "rules_cc/cc/runfiles/runfiles.h"

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace robodyna::parsers {
std::string ResolveRunfile(const std::string& name) {
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    std::string message;
    const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::Create(error ? "" : executable.string(), &message));
    if (!runfiles) throw std::runtime_error("Cannot locate parser runfiles: " + message);
    const auto path = runfiles->Rlocation(name);
    if (path.empty() || !std::filesystem::is_regular_file(path))
        throw std::runtime_error("Declared parser input is missing: " + name);
    return std::filesystem::absolute(path).string();
}

void ConfigureEmbeddedPackage(const std::string& manifest_runfile) {
    const auto root = std::filesystem::path(ResolveRunfile(manifest_runfile)).parent_path().string();
    // Deliberately select only the declared native package rather than an old
    // user-installed pychrono. CPython still provides its standard library.
    if (setenv("PYTHONPATH", root.c_str(), 1) || setenv("PYTHONNOUSERSITE", "1", 1) ||
        setenv("PYTHONDONTWRITEBYTECODE", "1", 1))
        throw std::runtime_error("Cannot configure the declared embedded Python package");
}

void ConfigureDemoData(const std::string& solidworks_model_runfile) {
    const auto data = std::filesystem::path(ResolveRunfile(solidworks_model_runfile)).parent_path().parent_path();
    chrono::SetChronoDataPath(data.string() + "/");
}
}  // namespace robodyna::parsers
