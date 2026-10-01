#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace robodyna::examples {
struct CaptureOptions {
    std::filesystem::path output, chrono_data;
    std::uint64_t steps = 0, capture_every = 0;
    double time_step_s = 0;
    bool headless = false;
    std::string executable, source_demo, source_demo_sha256;
    std::size_t capture_bytes = std::size_t{6} << 30;
};

// Native dt is supplied by the original demo, never changed by command-line options.
// Resolves and hashes the declared source runfile, validates the asset directory,
// and sets the existing Chrono asset path before a visual system is constructed.
CaptureOptions ReadCaptureOptions(int argc, char** argv, const std::string& source_demo, double native_dt);
std::size_t CaptureFrameCount(const CaptureOptions&);
void ValidateCaptureOptions(const CaptureOptions&);
bool CaptureTimeMatches(std::uint64_t step, double actual_time_s, double time_step_s) noexcept;
}  // namespace robodyna::examples
