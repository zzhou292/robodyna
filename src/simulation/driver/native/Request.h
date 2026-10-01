#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>

namespace robodyna::driver {
struct Packet {
    std::filesystem::path path;
    std::size_t bytes = 0;
    std::string sha256;
};
struct RunOptions {
    double duration_s = 0, fixed_dt_s = 0;
    std::size_t samples = 0;
    std::string contact_activity;
    bool stage_timing = false, capture_qeph_rejection = false, verify_initial_retry = false;
};
struct Resources {
    std::size_t rss_bytes = 0, archive_bytes = 0, artifact_file_bytes = 0, solid_worker_blocks = 0;
    double cooperative_maximum_elapsed_s = 0;
};
struct Request {
    std::map<std::string, std::filesystem::path> sources;
    Packet packets;
    RunOptions run;
    Resources resources;
    std::filesystem::path output, report, stop_file;
};

// Parse only; no source preparation, GPU allocation or output mutation.
Request ReadRequest(const std::filesystem::path&, const std::string& expected_sha256);
}  // namespace robodyna::driver
