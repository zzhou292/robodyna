#pragma once
#include "case/vehicle_run/Progress.h"
#include <optional>
#include <filesystem>
namespace crash::cases::vehicle_native_contact::test {
// Observation/control only. Neither option changes the physical horizon or dt.
struct PreviewControls {
    double maximum_elapsed_s = 0;
    std::filesystem::path stop_file;
    bool stage_timing = false;
    std::optional<std::size_t> artifact_file_bytes; // Absent preserves the owning RunConfig default.
};
PreviewControls ParsePreviewControls(const char* maximum_elapsed_s, const char* stop_file,
                                    const char* stage_timing = nullptr,
                                    const char* artifact_file_bytes = nullptr);
PreviewControls ReadPreviewControls();
vehicle_run::Control MakePreviewControl(const PreviewControls&);
} // namespace crash::cases::vehicle_native_contact::test
