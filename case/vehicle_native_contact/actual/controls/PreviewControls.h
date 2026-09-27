#pragma once
#include "case/vehicle_run/Progress.h"
#include <filesystem>
namespace crash::cases::vehicle_native_contact::test {
// Observation/control only. Neither option changes the physical horizon or dt.
struct PreviewControls {
    double maximum_elapsed_s = 0;
    std::filesystem::path stop_file;
};
PreviewControls ParsePreviewControls(const char* maximum_elapsed_s, const char* stop_file);
PreviewControls ReadPreviewControls();
vehicle_run::Control MakePreviewControl(const PreviewControls&);
} // namespace crash::cases::vehicle_native_contact::test
