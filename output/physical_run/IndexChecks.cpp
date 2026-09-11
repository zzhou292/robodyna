#include "Metadata.h"
#include <set>
#include <algorithm>
namespace crash::output::physical_run {
void CheckIndex(const records::Context& c,const Configuration& config,const Index& index) {
    const auto plan=records::activity::PlanWithActivity(c,config.request,"parent-activity.json").archive;
    Require(records::SameIdentity(c.identity(),config.identity) && c.point_layout_sha256()==config.point_layout_sha256 &&
        index.planned_intervals==config.request.intervals && index.accepted_intervals<=index.planned_intervals &&
        index.final.epoch==index.accepted_intervals && index.stop_reason.size()<=4096,
        "Physical run endpoint/source differs");
    records::CheckStamp(c,index.final);
    Require(index.horizon_complete ? (index.accepted_intervals==index.planned_intervals && index.stop_reason.empty()) :
        (index.accepted_intervals<index.planned_intervals && !index.stop_reason.empty()),"Physical prefix/completion claim differs");
    Require(!index.frames.empty() && index.frames.size()<=plan.frame_capacity && index.frames.front().stamp.epoch==0 &&
        records::SameStamp(index.frames.back().stamp,index.final),"Physical run lacks exact initial/final accepted frame");
    std::set<std::string> files;
    std::vector<std::uint64_t> epochs;
    unsigned off_cadence=0;
    for(const auto& f:index.frames) {
        records::CheckStamp(c,f.stamp);
        Require(f.stamp.epoch<=index.accepted_intervals && (epochs.empty() || f.stamp.epoch>epochs.back()),
            "Physical sampled epochs are duplicated or unordered");
        if(!std::binary_search(plan.frame_epochs.begin(),plan.frame_epochs.end(),f.stamp.epoch)) {
            ++off_cadence;Require(f.stamp.epoch==index.accepted_intervals,"Off-cadence frame is not the accepted prefix endpoint");
        }
        Require(files.insert(f.frame.file).second && files.insert(f.activity.file).second,"Physical frame files alias");
        FileDocument(f.frame);FileDocument(f.activity);epochs.push_back(f.stamp.epoch);
    }
    Require(off_cadence<=1,"Physical prefix frame reservation exceeded");
    for(auto epoch:plan.frame_epochs)if(epoch<=index.accepted_intervals)
        Require(std::binary_search(epochs.begin(),epochs.end(),epoch),"Physical accepted run omitted a scheduled frame");
    const auto chunks=interval::PlanChunks(index.planned_intervals,config.request.file_byte_cap);
    Require(index.segments.size()==index.accepted_intervals/chunks.rows_per_chunk+
        (index.accepted_intervals%chunks.rows_per_chunk!=0),"Physical accepted interval segments are incomplete");
}
} // namespace crash::output::physical_run
