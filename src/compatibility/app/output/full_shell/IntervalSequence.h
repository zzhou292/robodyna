#pragma once
#include "FullShellVisualizationSchema.h"
#include "IntervalChunkPlan.h"

namespace crash::output::full_shell {
struct IntervalLimits {
    std::uint64_t nodes=0,parents=0,plastic_points=0;
    double maximum_penetration=0,maximum_rotation=0,maximum_area_ratio=0,maximum_thickness_ratio=0;
    std::size_t file_bytes=kArtifactFileCap,host_bytes=128*1024*1024;
};
struct IntervalContext {
    Identity identity;
    double fixed_dt=0;
    std::uint64_t planned_intervals=0;
    IntervalLimits limits;
};
struct IntervalCursor {
    std::uint64_t epoch=0,attempt=0,first_contact=0,last_contact=0,contact_intervals=0;
    double time=0,velocity_time=0,base_time=0,maximum_plastic=0,plastic_work=0,wall_potential=0;
};
void CheckIntervalContext(const IntervalContext&);
void CheckCursor(const IntervalContext&,const IntervalCursor&);
bool SameCursor(const IntervalCursor&,const IntervalCursor&) noexcept;
FrameStamp IntervalStamp(const interval::Values&);
// Pure staged sequence check. Source authority is supplied by the caller.
// Q/T/contact participant stamps are checked by the live values factory before
// serialization; absent per-participant fields are not reconstructed here.
IntervalCursor AdvanceInterval(const IntervalContext&,const IntervalCursor&,const interval::Values&);
} // namespace crash::output::full_shell
