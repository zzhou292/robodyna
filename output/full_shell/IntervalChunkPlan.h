#pragma once
#include "IntervalValues.h"
#include "output/ArtifactInventory.h"

namespace crash::output::interval {
struct ChunkPlan {
    std::uint64_t intervals=0,rows_per_chunk=0;
    std::size_t chunks=0,row_bytes=0,total_bytes=0,file_byte_cap=0;
};
// Two typed files share one conservative row cap, including any explicit extra.
ChunkPlan PlanChunks(std::uint64_t intervals,std::size_t file_byte_cap=kArtifactFileCap,
                     std::size_t total_byte_cap=kArtifactMaximumTotalCap,std::size_t extra_row_bytes=0);
} // namespace crash::output::interval
