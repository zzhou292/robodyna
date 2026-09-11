#include "ArchiveState.h"
namespace crash::output::physical_frames {
namespace detail {
void FrameDestinations(const std::filesystem::path& root,const std::string& stem) {
    arrays::CheckRelativeName(stem);
    Require(stem.size()<=128 && stem.find('/')==std::string::npos,"Invalid physical frame stem");
    for(const auto* suffix:{".positions.bin",".plastic.bin",".frame.json",".activity.bin",".activity.json"})
        arrays::CheckedPath(root,stem+suffix,false);
}
void CheckPair(const records::Context& context,const records::FrameRecord& frame,
    const records::activity::ActivityRecord& activity) {
    records::CheckFrame(context,{frame.stamp,frame.position_xyz.data(),frame.position_xyz.size(),
        frame.plastic_points.data(),frame.plastic_points.size()});
    Require(records::SameIdentity(context.identity(),activity.context().identity()) &&
        context.point_layout_sha256()==activity.context().point_layout_sha256() &&
        records::SameStamp(frame.stamp,activity.stamp()),"Frame/activity accepted source or phase differs");
}
}
} // namespace crash::output::physical_frames
