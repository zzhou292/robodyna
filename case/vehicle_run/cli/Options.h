#pragma once
#include "../Config.h"
#include "../source/OriginalPaths.h"
#include <optional>
namespace crash::cases::vehicle_run::cli {
struct Options {
    OriginalPaths source;
    std::filesystem::path output,stop_file;
    Config config;
    double gap_m=.02;
    // Absent values preserve the owning wall-case defaults.
    std::optional<double> wall_stiffness_n_m3,penetration_limit_m;
    std::uint64_t diagnostic_intervals=0;
    double maximum_elapsed_s=0;
    std::uint64_t run_id=0;
    bool forecast_only=false;
};
Options Parse(int argc,const char* const* argv);
const char* Usage() noexcept;
}
