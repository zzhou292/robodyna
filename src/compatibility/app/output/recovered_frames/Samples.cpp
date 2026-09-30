#include "Internal.h"
#include <algorithm>

namespace crash::output::recovered_frames {
void CheckFrames(const records::Context& context,const run::Configuration& config,
        const std::vector<run::FrameFiles>& frames) {
    const auto schedule=records::activity::PlanWithActivity(context,config.request,"parent-activity.json").archive.frame_epochs;
    Require(!frames.empty() && frames.size()<=schedule.size(),"Recovered samples exceed original cadence");
    for(std::size_t i=0;i<frames.size();++i) {
        const auto& frame=frames[i];
        records::CheckStamp(context,frame.stamp);
        const auto stem="frame-"+std::to_string(schedule[i]);
        Require(frame.stamp.epoch==schedule[i] && frame.frame.file==stem+".frame.json" &&
            frame.activity.file==stem+".activity.json","Recovered samples omit/reorder original cadence");
        if(i)Require(frame.stamp.attempt>frames[i-1].stamp.attempt && frame.stamp.time>frames[i-1].stamp.time,
            "Recorded sampled attempts/time do not increase");
    }
    // Missing interval history remains unavailable even if the final scheduled
    // image happened to finish. No requested-duration completion is inferred.
}
std::vector<run::FrameFiles> ScanFrames(const std::filesystem::path& root,const records::Context& context,
        const run::Configuration& config) {
    const auto schedule=records::activity::PlanWithActivity(context,config.request,"parent-activity.json").archive.frame_epochs;
    std::vector<run::FrameFiles> frames;
    frames.reserve(schedule.size());
    for(auto epoch:schedule) {
        const auto stem="frame-"+std::to_string(epoch);
        if(!std::filesystem::exists(root/(stem+".frame.json")) ||
            !std::filesystem::exists(root/(stem+".activity.json")))break;
        const auto frame=InspectFile(root,stem+".frame.json",run::MetadataCap);
        const auto activity=InspectFile(root,stem+".activity.json",records::activity::MetadataByteCap);
        const auto description=records::ParseFrameDocument(context,
            array_json::Parse(run::ReadFile(root,frame,run::MetadataCap),run::MetadataCap));
        // Payload digests, finite fields, phase and packed activity padding are
        // validated by the existing typed readers before recording a sample.
        records::ReadFrame(root,context,frame,description.stamp);
        records::activity::ReadActivity(root,context,activity,description.stamp);
        frames.push_back({description.stamp,frame,activity});
    }
    CheckFrames(context,config,frames);
    return frames;
}
} // namespace crash::output::recovered_frames
