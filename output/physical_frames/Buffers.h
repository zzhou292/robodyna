#pragma once
#include "output/full_shell/activity/ActivityRecord.h"
#include "FieldTypes.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/qbat/QbatBatch.h"
#include <optional>
namespace crash::output::physical_frames {
namespace records=full_shell;
struct Limits {
    std::size_t host_bytes=512u<<20;
    records::RecordLimits records;
};
struct Forecast {
    std::size_t retained_bytes=0, temporary_bytes=0, peak_bytes=0;
    std::size_t physical_nodes=0, layered_rows=0, qbat_rows=0;
};
namespace detail {
Forecast PlanBuffers(const records::Context&,std::size_t physical_nodes,
    std::size_t qeph,std::size_t t3,std::size_t qbat,std::size_t mapping_bytes,Limits);
// Explicit single-QEPH suffix. Legacy PlanBuffers keeps exact total equality.
Forecast PlanBuffersWithEnvironment(const records::Context&,std::size_t physical_nodes,
    FamilyCounts physical,FamilyCounts rendered,std::size_t mapping_bytes,Limits);
// Unpublished fields may be incomplete after a rejected readback. Frame and
// packed activity become visible through the same selector only after Finish.
struct FrameBuffers {
    explicit FrameBuffers(const records::Context&);
    records::FrameRecord frames[2];
    std::optional<records::activity::ActivityRecord> activity[2];
    std::vector<std::uint8_t> flags;
    unsigned selected=0;
    bool available=false;
    records::FrameRecord& Staging() noexcept {return frames[1-selected];}
    void Finish(const records::Context&,records::FrameStamp);
};
} // namespace detail
} // namespace crash::output::physical_frames
