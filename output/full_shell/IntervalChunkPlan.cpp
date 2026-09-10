#include "IntervalChunkPlan.h"

namespace crash::output::interval {
ChunkPlan PlanChunks(std::uint64_t intervals,std::size_t file_cap,std::size_t total_cap,std::size_t extra) {
    Require(intervals&&file_cap>=RowBytes&&file_cap<=kArtifactFileCap&&
        total_cap&&total_cap<=kArtifactMaximumTotalCap&&extra<=file_cap-RowBytes,
        "Invalid interval chunk capacity");
    ChunkPlan result;
    result.intervals=intervals;
    result.file_byte_cap=file_cap;
    result.row_bytes=RowBytes+extra;
    Require(intervals<=total_cap/result.row_bytes,"Interval ledger exceeds total byte capacity");
    result.total_bytes=static_cast<std::size_t>(intervals)*result.row_bytes;
    result.rows_per_chunk=file_cap/result.row_bytes;
    result.chunks=static_cast<std::size_t>(intervals/result.rows_per_chunk+(intervals%result.rows_per_chunk!=0));
    Require(result.chunks<=64,"Interval ledger exceeds 64 chunks");
    return result;
}
} // namespace crash::output::interval
