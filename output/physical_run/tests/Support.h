#pragma once
#include "../Replay.h"
#include "../IntervalIO.h"
#include "../../full_shell/tests/TestSupport.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run::test {
namespace ft=full_shell::test;
inline records::Context Context() {
    const records::ParentPoints parents[]{
        {71,201,2,1,0,records::PlasticField::NotApplicable},
        {72,202,9,2,1,records::PlasticField::NativeEquivalentPlasticStrain},
        {73,203,2,1,3,records::PlasticField::NativeEquivalentPlasticStrain},
        {74,204,9,3,4,records::PlasticField::NativeEquivalentPlasticStrain}};
    auto id=ft::Id();id.owner=UINT64_C(9007199254740995);
    return records::Context::Create(id,5,parents,4,.125);
}
inline Configuration Config(const records::Context& c,Profile p={false,true},unsigned planned=4) {
    records::PlanRequest r;
    r.nodes=c.nodes();r.parents=c.parents().size();r.plastic_points=c.points();r.frames=3;r.intervals=planned;
    r.fixed_dt=c.fixed_dt();r.requested_duration=planned*c.fixed_dt();r.static_byte_reserve=4u<<20;
    r.static_files={{"manifest.json",MetadataCap},{"frame-index.json",MetadataCap},{"configuration.json",MetadataCap}};
    return {c.identity(),p,r,c.point_layout_sha256()};
}
inline Values Row(const records::Context& c,std::uint64_t epoch,std::uint64_t attempt=0,bool bound=true) {
    const double base=(epoch-1)*c.fixed_dt();
    Values v{c.identity().owner,{epoch,epoch-1,attempt?attempt:2*epoch,base+c.fixed_dt(),base,
        base+.5*c.fixed_dt(),epoch==1?.5*c.fixed_dt():c.fixed_dt()},std::nullopt};
    if(bound)v.structural_limit_s=.2;
    return v;
}
inline FrameFiles Frame(const std::filesystem::path& root,const records::Context& c,records::FrameStamp stamp) {
    records::FrameRecord f{stamp,{0.,-0.,1.,1.,0.,1.,0.,1.,1.,1.,1.,1.,.5,.5,.75},std::vector<double>(c.points())};
    for(std::size_t i=0;i<f.plastic_points.size();++i)f.plastic_points[i]=(i+1)*.001*stamp.epoch;
    const auto stem="frame-"+std::to_string(stamp.epoch);
    const auto file=records::WriteFrame(root,stem,c,{stamp,f.position_xyz.data(),f.position_xyz.size(),f.plastic_points.data(),f.plastic_points.size()});
    const std::uint8_t active[]{1,1,static_cast<std::uint8_t>(stamp.epoch<3),1};
    const auto activity=records::activity::ActivityRecord::Create(c,{c.identity(),c.point_layout_sha256(),stamp,active,4},stamp);
    return {stamp,file,records::activity::WriteActivity(root,stem,activity)};
}
inline Index WriteRun(const std::filesystem::path& root,const records::Context& c,const Configuration& config,
    std::uint64_t accepted) {
    IntervalWriter writer(root,c,config.profile,config.request.intervals,config.request.file_byte_cap,64u<<20);
    Index index;index.planned_intervals=config.request.intervals;index.accepted_intervals=accepted;
    index.horizon_complete=accepted==index.planned_intervals;
    index.stop_reason=index.horizon_complete?"":"test accepted-prefix stop";
    index.frames.push_back(Frame(root,c,{}));
    const auto plan=records::activity::PlanWithActivity(c,config.request,"parent-activity.json");
    for(std::uint64_t epoch=1;epoch<=accepted;++epoch) {
        const auto row=Row(c,epoch,0,config.profile.structural_limit);writer.Append(row);index.final=row.stamp;
        if(epoch==accepted || std::binary_search(plan.archive.frame_epochs.begin(),plan.archive.frame_epochs.end(),epoch))
            index.frames.push_back(Frame(root,c,row.stamp));
    }
    index.segments=writer.Finish();CheckIndex(c,config,index);return index;
}
} // namespace crash::output::physical_run::test
