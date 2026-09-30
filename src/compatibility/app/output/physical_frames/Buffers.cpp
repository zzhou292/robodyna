#include "Buffers.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::output::physical_frames::detail {
Forecast PlanBuffers(const records::Context& context,std::size_t physical,
    std::size_t qeph,std::size_t t3,std::size_t qbat,std::size_t mapping,Limits limits) {
    Require(limits.host_bytes && limits.host_bytes<=512u<<20 && physical>=context.nodes() &&
        physical<=524288 && qeph<=524288 && t3<=524288 && qbat<=524288 &&
        qeph+t3+qbat==context.parents().size(),"Invalid physical capture dimensions or host cap");
    Forecast out;
    out.physical_nodes=physical;
    out.layered_rows=std::max(qeph,t3);
    out.qbat_rows=qbat;
    tl::util::BoundedArenaLayout budget(limits.host_bytes);
    tl::util::ArenaRegion unused;
    Require(budget.Append<std::byte>(mapping,unused) &&
        budget.Append<std::byte>(context.retained_payload_bytes(),unused) &&
        budget.Append<std::byte>(8192,unused) &&
        budget.Append<double>(6*context.nodes()+2*context.points()+6*physical,unused) &&
        budget.Append<tl::fea::ShellBatchLayeredSection>(out.layered_rows,unused) &&
        budget.Append<tl::fea::qbat::BatchResult>(qbat,unused) &&
        budget.Append<std::uint8_t>(context.parents().size()+std::max(out.layered_rows,qbat),unused) &&
        budget.Append<std::uint64_t>(2*((context.parents().size()+63)/64),unused),
        "Complete accepted capture buffers exceed host cap");
    out.retained_bytes=budget.bytes();
    // Covers ActivityRecord packing/metadata temporaries (its Context and input
    // spans are already owned above), and the larger two-array frame codec.
    const auto activity=records::activity::detail::Preflight(context,{limits.host_bytes});
    const auto codec=sizeof(double)*(3*context.nodes()+context.points())+16*records::FrameMetadataByteCap;
    out.temporary_bytes=std::max(activity,codec);
    Require(budget.Append<std::byte>(out.temporary_bytes,unused),"Accepted capture/codec peak exceeds host cap");
    out.peak_bytes=budget.bytes();
    return out;
}
Forecast PlanBuffersWithEnvironment(const records::Context& context,std::size_t physical_nodes,
    FamilyCounts physical,FamilyCounts rendered,std::size_t mapping,Limits limits) {
    Require(rendered.qeph<524288 && physical.qeph==rendered.qeph+1 && physical.t3==rendered.t3 &&
        physical.qbat==rendered.qbat && physical_nodes>=context.nodes()+4,
        "Combined capture requires exactly one QEPH/four-node environment suffix");
    auto result=PlanBuffers(context,physical_nodes,rendered.qeph,rendered.t3,rendered.qbat,mapping,limits);
    const auto layered=std::max(physical.qeph,physical.t3);
    const auto flags=std::max(layered,physical.qbat)-std::max(result.layered_rows,result.qbat_rows);
    const auto extra=(layered-result.layered_rows)*sizeof(tl::fea::ShellBatchLayeredSection)+flags+
        (flags?alignof(std::uint64_t)-1:0); // Added flags can shift the following packed activity alignment.
    Require(result.peak_bytes<=limits.host_bytes && extra<=limits.host_bytes-result.peak_bytes,
        "Complete environment family readback exceeds capture cap");
    result.layered_rows=layered;result.retained_bytes+=extra;result.peak_bytes+=extra;
    return result;
}
FrameBuffers::FrameBuffers(const records::Context& context):flags(context.parents().size()) {
    for(auto& frame:frames) {
        frame.position_xyz.resize(3*context.nodes());
        frame.plastic_points.resize(context.points());
    }
}
void FrameBuffers::Finish(const records::Context& context,records::FrameStamp stamp) {
    auto& next=Staging();
    records::CheckFrame(context,{stamp,next.position_xyz.data(),next.position_xyz.size(),
        next.plastic_points.data(),next.plastic_points.size()});
    auto packed=records::activity::ActivityRecord::Create(context,
        {context.identity(),context.point_layout_sha256(),stamp,flags.data(),flags.size()},stamp);
    next.stamp=stamp;
    activity[1-selected].emplace(std::move(packed));
    selected=1-selected;
    available=true;
}
} // namespace crash::output::physical_frames::detail
