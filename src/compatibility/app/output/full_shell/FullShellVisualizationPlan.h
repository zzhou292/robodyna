#pragma once
#include "FullShellVisualizationSchema.h"

namespace crash::output::full_shell {
struct FileReservation {std::string file;std::size_t bytes=0;};
struct PlanRequest {
    std::size_t nodes=0,parents=0,plastic_points=0,frames=100;
    std::uint64_t intervals=0;
    double fixed_dt=0,requested_duration=0;
    // All known static files, including final manifest/index/configuration, must
    // be listed. The unused reserve is charged too; this list is not proof of
    // completeness of a future vehicle publisher.
    std::vector<FileReservation> static_files;
    std::size_t static_byte_reserve=StaticReserveBytes;
    std::size_t extra_interval_bytes=0,extra_frame_bytes=0;
    std::size_t total_byte_cap=TotalByteCap,file_byte_cap=kArtifactFileCap;
};
struct Plan {
    std::size_t position_bytes=0,plastic_bytes=0,frame_capacity=0,frame_bytes=0;
    std::size_t interval_row_bytes=0,interval_bytes=0,interval_chunks=0;
    std::size_t static_declared_bytes=0,forecast_bytes=0,forecast_files=0;
    std::uint64_t rows_per_chunk=0;
    std::vector<std::uint64_t> frame_epochs;
};
// No I/O or borrowed-value reads. Exact active counts, one prefix-frame reserve,
// explicit non-frame reserve, and all optional records are charged before use.
Plan PlanArchive(const PlanRequest&);
} // namespace crash::output::full_shell
