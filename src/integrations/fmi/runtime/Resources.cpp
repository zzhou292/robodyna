#include "Resources.h"
#include "rules_cc/cc/runfiles/runfiles.h"

#include <filesystem>
#include <memory>
#include <stdexcept>

namespace robodyna::fmi {
std::string ArchiveRunfile(const std::string& name) {
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    std::string message;
    const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::Create(error ? "" : executable.string(), &message));
    if (!runfiles) {
        throw std::runtime_error("Cannot locate FMI driver runfiles: " + message);
    }
    const auto path = runfiles->Rlocation(name);
    if (path.empty() || !std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("Declared FMU is missing from driver runfiles: " + name);
    }
    return std::filesystem::absolute(path).string();
}
}  // namespace robodyna::fmi
