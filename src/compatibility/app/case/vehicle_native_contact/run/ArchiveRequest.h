#pragma once
#include "case/vehicle_run/Config.h"
#include "output/physical_run/RunArchive.h"
namespace crash::cases::vehicle_native_contact::run_detail {
// Forecast and writer must use the same explicit output resource request.
// Changing chunk size never changes physical records or their source authority.
output::full_shell::source::BundleRequest MakeArchiveRequest(
    const output::full_shell::Context&, const vehicle_run::Horizon&,
    std::size_t samples, std::size_t archive_bytes,
    std::size_t artifact_file_bytes = output::kArtifactFileCap);
}
