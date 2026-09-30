#include "ArchiveRequest.h"
namespace crash::cases::vehicle_native_contact::run_detail {
output::full_shell::source::BundleRequest MakeArchiveRequest(
    const output::full_shell::Context& context, const vehicle_run::Horizon& horizon,
    std::size_t samples, std::size_t archive_bytes, std::size_t artifact_file_bytes) {
    output::Require(artifact_file_bytes >= output::full_shell::IntervalCoreBytes &&
                        artifact_file_bytes <= output::kArtifactFileCap,
                    "Native output artifact byte limit is outside the supported range");
    auto request = output::physical_run::MakeEnvironmentRequest(context, horizon.intervals,
        horizon.requested_duration_s, samples, archive_bytes);
    request.archive.file_byte_cap = artifact_file_bytes;
    return request;
}
}
