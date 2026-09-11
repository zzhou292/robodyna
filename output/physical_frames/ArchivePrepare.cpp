#include "ArchiveState.h"
#include "lib_utils/BoundedArena.h"
namespace crash::output::physical_frames {
Archive Archive::Prepare(const source::PreparedSourceMapping& mapping,const records::Context& context,
    source::BundleRequest request,std::size_t cap) {
    Require(cap && cap<=256u<<20,"Invalid accepted archive startup cap");
    const auto expected=mapping.MakeFrameContext(context.identity(),context.fixed_dt(),context.limits());
    Require(expected.point_layout_sha256()==context.point_layout_sha256() && expected.points()==context.points(),
        "Archive context differs from original source point mapping");
    // Validate whole-run cadence and optional sidecar costs before source copying.
    const std::string declaration="parent-activity.json";
    const auto early=records::activity::PlanWithActivity(context,request.archive,declaration);
    const auto& input=mapping.source().data();
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion ignored;
    Require(budget.Append<std::byte>(2*input.inputs.source_member.bytes,ignored) &&
        budget.Append<std::byte>(context.retained_payload_bytes()+2*records::FrameMetadataByteCap,ignored) &&
        budget.Append<double>(3*context.nodes()+context.points(),ignored) &&
        budget.Append<std::uint64_t>(early.archive.frame_capacity,ignored) &&
        budget.Append<std::byte>(4u<<20,ignored),"Archive source/serialization startup exceeds host cap");
    // Include the declaration in the source bundle's static feasibility check.
    auto source_request=request;
    source_request.archive.static_files.push_back({declaration,records::activity::MetadataByteCap});
    auto bundle=source::PreparedSourceBundle::Prepare(mapping,source_request);
    for(const auto& reservation:bundle.reservations()) request.archive.static_files.push_back(reservation);
    auto complete=records::activity::PlanWithActivity(context,request.archive,declaration);
    auto next=std::make_unique<Data>(context,std::move(bundle),std::move(complete));
    next->host_bytes=budget.bytes();
    next->intervals=request.archive.intervals;
    next->written.reserve(next->plan.archive.frame_capacity);
    return Archive(std::move(next));
}
Archive::Archive(std::unique_ptr<Data> data):data_(std::move(data)) {}
Archive::~Archive()=default;
Archive::Archive(Archive&&) noexcept=default;
Archive& Archive::operator=(Archive&&) noexcept=default;
const source::PreparedSourceBundle& Archive::source_bundle() const noexcept {return data_->bundle;}
const records::activity::ActivityPlan& Archive::plan() const noexcept {return data_->plan;}
std::size_t Archive::startup_host_bytes() const noexcept {return data_->host_bytes;}
std::size_t Archive::written_frames() const noexcept {return data_->written.size();}
} // namespace crash::output::physical_frames
