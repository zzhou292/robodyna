#include "SourceAssemblyBinaryFrameState.h"
#include "ComponentContext.h"
#include "lib_utils/BoundedArena.h"

namespace crash::output::assembly::binary {
SourceAssemblyBinaryFrames::SourceAssemblyBinaryFrames(const dynamics::SourceAssemblyWallCase& run,
        visual::Identity identity,std::uint64_t asset,Limits limits) {
    wall_fields::CheckCase(run);
    const auto& source=run.bindings()->source().data();
    Require(limits.frame_bytes&&limits.frame_bytes<=1024*1024&&source.nodes.size()<=2048&&
        source.parents.size()<=1024,"Component binary frame startup capacity exceeded");
    tl::util::BoundedArenaLayout budget(limits.frame_bytes);tl::util::ArenaRegion ignored;
    Require(budget.Append<double>(6*source.nodes.size(),ignored)&&
        budget.Append<double>(6*source.parents.size(),ignored),"Component binary frame buffers exceed startup budget");
    auto next=std::make_unique<Impl>();
    const auto report=next->capture.Initialize(*run.owner(),*run.bindings(),identity,asset,limits.capture);
    Require(report.status==visual::Status::Ok,report.message);
    const auto& settings=*run.setup()->settings();
    next->context.emplace(ComponentContext(*run.bindings(),*next->capture.mapping(),
        settings.configuration_id,settings.qualification_id,run.owner()->accepted().fixed_dt,limits.records));
    for(auto& frame:next->frames) {
        frame.position_xyz.resize(3*next->context->nodes());
        frame.plastic_points.resize(next->context->points());
    }
    next->bytes=budget.bytes();impl_=std::move(next);
}
SourceAssemblyBinaryFrames::~SourceAssemblyBinaryFrames()=default;
void SourceAssemblyBinaryFrames::Capture(dynamics::SourceAssemblyWallCase& run) {
    auto& state=*impl_;
    const auto result=run.CaptureAccepted(state.capture);
    Require(bool(result),result.message);
    const auto view=wall_fields::AcceptedFrameView(run,state.capture);
    auto& staged=state.frames[1-state.published];
    detail::StageFrame(*state.context,view,staged);
    state.published=1-state.published;
    state.available=true;
}
const records::Context& SourceAssemblyBinaryFrames::context() const noexcept {return *impl_->context;}
const SourceAssemblySurface& SourceAssemblyBinaryFrames::mapping() const noexcept {return *impl_->capture.mapping();}
const records::FrameRecord* SourceAssemblyBinaryFrames::frame() const noexcept {
    return impl_->available?&impl_->frames[impl_->published]:nullptr;
}
std::size_t SourceAssemblyBinaryFrames::frame_bytes() const noexcept {return impl_->bytes;}
records::RecordFile SourceAssemblyBinaryFrames::Write(const std::filesystem::path& root,const std::string& stem) const {
    const auto* accepted=frame();Require(accepted,"Component has no captured accepted binary frame");
    return records::WriteFrame(root,stem,context(),{accepted->stamp,accepted->position_xyz.data(),
        accepted->position_xyz.size(),accepted->plastic_points.data(),accepted->plastic_points.size()});
}
} // namespace crash::output::assembly::binary
