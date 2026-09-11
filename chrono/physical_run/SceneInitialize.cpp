#include "SceneState.h"
#include "chrono/ReplayVisuals.h"
#include "chrono/ReplayDisplayGeometry.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/physics/ChBody.h"
namespace crash::visual::physical_run {
ReplaySceneReport Scene::Initialize(const output::physical_run::Replay& replay,SceneOptions options) {
    if(impl_) return {ReplaySceneStatus::AlreadyInitialized,"Physical scene is already initialized"};
    try {
        const auto budget=Forecast(replay.peak_host_bytes(),replay.context().nodes(),replay.context().parents().size(),
            replay.mapping().triangles(),replay.context().points(),options);
        const auto scan=Scan(replay);
        auto next=std::make_unique<Impl>(replay);
        next->budget=budget;
        next->plastic_maximum=scan.plastic_maximum;
        // Fit the complete archived motion and wall with room around the model.
        output::Require(MakeBoundsCamera(scan.low,scan.high,{-1.,-1.,.45},.85,ReplayVertical::Z,
            options.view,next->camera),"Invalid physical replay camera bounds");
        full_shell::FrameGeometryOptions geometry;
        geometry.geometry=ReplayGeometryLimits::Vehicle();
        geometry.colors=options.colors;
        geometry.parent_activity=true;
        geometry.plastic_strain_maximum=scan.plastic_maximum;
        auto report=next->geometry.Initialize(replay.mapping(),replay.context(),geometry);
        if(report.status!=ReplaySceneStatus::Ok) return report;
        const auto initial=replay.ReadSample(0);
        report=next->geometry.Update(initial.frame,initial.activity,replay.index().frames[0].stamp);
        if(report.status!=ReplaySceneStatus::Ok) return report;
        // This is the adapter's own non-const presentation object. No archive
        // bytes or physical owner are made mutable by binding the visual shape.
        auto mesh=std::const_pointer_cast<chrono::ChTriangleMeshConnected>(next->geometry.mesh());
        next->shape=MakeReplayShape(mesh,true,options.wireframe,
            next->geometry.color_mode()!=ReplayColorMode::Uniform,false);
        next->system.SetGravitationalAcceleration(chrono::VNULL);
        next->system.AddBody(MakeReplayCarrier("physical accepted shell surface",next->shape));
        if(const auto wall=replay.wall_mesh()) {
            auto shape=MakeReplayShape(CopyReplayGeometry(*wall),false,true);
            next->system.AddBody(MakeReplayCarrier("authenticated selected wall",shape));
        }
        next->stamp={0,replay.context().identity().owner,initial.frame.stamp.epoch,initial.frame.stamp.time};
        next->system.SetChTime(next->stamp.time);
        impl_=std::move(next);
        return {ReplaySceneStatus::Ok,"Physical accepted initial sample published"};
    } catch(const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit,"Physical scene allocation failed"};
    } catch(const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame,"Physical replay source, sample or display configuration rejected"};
    }
}
} // namespace crash::visual::physical_run
