#include "ActivityRecord.h"
#include "Internal.h"

namespace crash::output::full_shell::activity {
ActivityPlan PlanWithActivity(const Context& c,const PlanRequest& request,const std::string& sidecar) {
    Require(request.nodes==c.nodes()&&request.parents==c.parents().size()&&request.plastic_points==c.points()&&
        Bits(request.fixed_dt)==Bits(c.fixed_dt()),"Activity plan differs from record context");
    Require(!request.extra_frame_bytes,"Activity profile cannot combine unrelated extra-frame records");
    ActivityPlan result;result.packed_frame_bytes=arrays::ByteCount(detail::Layout(request.parents),
        {request.file_byte_cap,UINT32_MAX,64});
    Require(result.packed_frame_bytes<=request.file_byte_cap&&MetadataByteCap<=request.file_byte_cap,
        "Activity frame reservation exceeds file capacity");
    auto augmented=request;augmented.extra_frame_bytes=result.packed_frame_bytes;
    augmented.static_files.push_back({sidecar,MetadataByteCap});
    result.archive=PlanArchive(augmented);
    // Metadata is a second bounded file, not bytes appended to the bit array.
    Require(result.archive.frame_capacity<=(request.total_byte_cap-result.archive.forecast_bytes)/MetadataByteCap,
        "Activity metadata exceeds total archive capacity");
    result.archive.forecast_bytes+=result.archive.frame_capacity*MetadataByteCap;
    result.archive.frame_bytes+=MetadataByteCap;
    // Existing optional-frame accounting reserves one file; this profile has a
    // packed array AND metadata. Count its second file without changing V1.
    Require(result.archive.frame_capacity<=kArtifactInventoryCap-result.archive.forecast_files,
        "Activity archive exceeds file inventory capacity");
    result.archive.forecast_files+=result.archive.frame_capacity;return result;
}
} // namespace crash::output::full_shell::activity
