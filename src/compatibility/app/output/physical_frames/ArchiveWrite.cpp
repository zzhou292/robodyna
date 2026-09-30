#include "ArchiveState.h"
#include <algorithm>
namespace crash::output::physical_frames {
FrameFiles Archive::Write(const std::filesystem::path& root,const std::string& stem,
    const records::FrameRecord& frame,const records::activity::ActivityRecord& activity) {
    auto& s=*data_;
    Require(!s.io_failed,"Archive has incomplete I/O evidence; use a new destination and archive handle");
    detail::CheckPair(s.context,frame,activity);
    const auto epoch=frame.stamp.epoch;
    Require(epoch<=s.intervals && s.written.size()<s.plan.archive.frame_capacity &&
        (s.written.empty() || epoch>s.written.back()),"Frame cadence/capacity or accepted ordering differs");
    const auto& schedule=s.plan.archive.frame_epochs;
    const bool scheduled=std::binary_search(schedule.begin(),schedule.end(),epoch);
    Require(scheduled || !s.prefix_used,"The single off-cadence accepted-prefix reserve is consumed");
    detail::FrameDestinations(root,stem);
    FrameFiles out;
    out.stamp=frame.stamp;
    try {
        out.frame=records::WriteFrame(root,stem,s.context,{frame.stamp,frame.position_xyz.data(),frame.position_xyz.size(),
            frame.plastic_points.data(),frame.plastic_points.size()});
        out.activity=records::activity::WriteActivity(root,stem,activity);
    } catch(...) {
        // Any partial files consume evidence/space. Do not offer an unbounded
        // sequence of retries under a frame reservation after an I/O failure.
        s.io_failed=true;
        throw;
    }
    s.written.push_back(epoch);
    s.prefix_used|=!scheduled;
    return out;
}
} // namespace crash::output::physical_frames
