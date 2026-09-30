#include "Replay.h"
#include "IntervalIO.h"
namespace crash::output::physical_run {
void ValidateRecords(const std::filesystem::path& root,const records::Context& c,const Configuration& config,
    const Index& index,std::size_t cap) {
    CheckIndex(c,config,index);
    std::size_t next_frame=1;
    const auto final=ReadIntervals(root,c,config.profile,index.planned_intervals,index.accepted_intervals,index.segments,
        config.request.file_byte_cap,cap,[&](const Values& row) {
            if(next_frame<index.frames.size() && index.frames[next_frame].stamp.epoch==row.stamp.epoch) {
                Require(records::SameStamp(index.frames[next_frame].stamp,row.stamp),"Physical sampled phase differs from accepted ledger");
                ++next_frame;
            }
        });
    Require(records::SameStamp(final.last,index.final) && next_frame==index.frames.size(),"Physical run final/index phase differs");
    for(const auto& f:index.frames) {
        const auto frame=records::ReadFrame(root,c,f.frame,f.stamp);
        const auto active=records::activity::ReadActivity(root,c,f.activity,f.stamp);
        Require(records::SameStamp(frame.stamp,active.stamp()),"Physical sampled activity phase differs");
    }
}
} // namespace crash::output::physical_run
