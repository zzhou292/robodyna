#include "RunState.h"
#include "output/physical_frames/ArchiveState.h"
#include <algorithm>
namespace crash::output::physical_run {
void RunArchive::Append(const AcceptedInterval& row) {
    auto& s=*data_;
    Require(!failed() && !s.closed && !s.prefix_sample,"Physical run is closed or poisoned");
    Require(records::SameIdentity(row.identity(),s.context.identity()) && SameProfile(row.profile(),s.configuration.profile),
        "Accepted interval belongs to another physical run/profile");
    Require(s.configuration.wall==bool(s.wall) && s.wall.Matches(row.wall_),
        "Accepted interval differs from the actual archived wall setup or has no loaded contact");
    const auto epoch=s.intervals->sequence().last.epoch;
    const auto& schedule=s.forecast.archive.archive.frame_epochs;
    if(std::binary_search(schedule.begin(),schedule.end(),epoch))
        Require(!s.index.frames.empty() && s.index.frames.back().stamp.epoch==epoch,"Capture the scheduled accepted frame before advancing the ledger");
    // Formatting validation is retryable; a failed chunk write poisons the writer.
    s.intervals->Append(row.values());
}
void RunArchive::Sample(const records::FrameRecord& frame,const records::activity::ActivityRecord& activity) {
    auto& s=*data_;
    Require(!failed() && !s.closed && !s.prefix_sample,"Physical run is closed or poisoned");
    Require(records::SameStamp(frame.stamp,s.intervals->sequence().last),"Sample is not the latest logged accepted endpoint");
    physical_frames::detail::CheckPair(s.context,frame,activity);
    const auto stem="frame-"+std::to_string(frame.stamp.epoch);
    physical_frames::detail::FrameDestinations(s.root,stem);
    physical_frames::FrameFiles saved;
    try {saved=s.frames.Write(s.root,stem,frame,activity);} catch(...) {s.failed=true;throw;}
    s.index.frames.push_back({saved.stamp,std::move(saved.frame),std::move(saved.activity)});
    const auto& schedule=s.forecast.archive.archive.frame_epochs;
    s.prefix_sample=!std::binary_search(schedule.begin(),schedule.end(),frame.stamp.epoch);
}
records::RecordFile RunArchive::Close(bool prefix,const std::string& reason) {
    auto& s=*data_;Require(!failed() && !s.closed,"Physical run is closed or poisoned");
    Require(s.configuration.wall==bool(s.manifest.wall),"Physical wall receipt is incomplete");
    s.index.final=s.intervals->sequence().last;s.index.accepted_intervals=s.index.final.epoch;
    Require(prefix ? (s.index.accepted_intervals<s.index.planned_intervals && !reason.empty() && reason.size()<=4096) :
        (s.index.accepted_intervals==s.index.planned_intervals && reason.empty()),"Physical completion/prefix endpoint differs");
    Require(!s.index.frames.empty() && records::SameStamp(s.index.frames.back().stamp,s.index.final),
        "Physical prefix needs the exact final accepted frame");
    s.index.horizon_complete=!prefix;s.index.stop_reason=reason;
    arrays::CheckedPath(s.root,"frame-index.json",false);arrays::CheckedPath(s.root,"manifest.json",false);
    try {
        s.index.segments=s.intervals->Finish();CheckIndex(s.context,s.configuration,s.index);
        s.manifest.index=WriteDocument(s.root,"frame-index.json",IndexDocument(s.configuration,s.index),MetadataCap);
        s.manifest.inventory=Inventory(s.root,s.configuration.request.total_byte_cap-MetadataCap,s.configuration.wall);
        CheckReferencedInventory(s.root,s.context,s.index,s.manifest);
        const auto result=WriteDocument(s.root,"manifest.json",ManifestDocument(s.manifest),MetadataCap);
        s.closed=true;return result;
    } catch(...) {s.failed=true;throw;}
}
records::RecordFile RunArchive::Finish() {return Close(false,{});}
records::RecordFile RunArchive::FinishPrefix(const std::string& reason) {return Close(true,reason);}
} // namespace crash::output::physical_run
