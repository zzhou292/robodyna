#include "SceneState.h"
#include <stdexcept>
namespace crash::visual::physical_run {
Scene::Scene()=default;
Scene::~Scene()=default;
ReplaySceneReport Scene::Publish(std::size_t sample) {
    if(!impl_) return {ReplaySceneStatus::NotInitialized,"Physical scene is not initialized"};
    try {
        const auto next=impl_->reader.ReadSample(sample);
        const auto report=impl_->geometry.Update(next.frame,next.activity,impl_->reader.index().frames[sample].stamp);
        if(report.status!=ReplaySceneStatus::Ok) return report;
        impl_->stamp={sample,impl_->reader.context().identity().owner,next.frame.stamp.epoch,next.frame.stamp.time};
        impl_->system.SetChTime(next.frame.stamp.time);
        return {ReplaySceneStatus::Ok,"Exact accepted sample published"};
    } catch(const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit,"Physical seek allocation failed"};
    } catch(const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame,"Physical seek failed before display publication"};
    }
}
chrono::ChSystem& Scene::system() {
    if(!impl_) throw std::logic_error("Physical scene is not initialized");
    return impl_->system;
}
const ReplayCamera* Scene::camera() const noexcept {return impl_?&impl_->camera:nullptr;}
const ReplayStamp* Scene::stamp() const noexcept {return impl_?&impl_->stamp:nullptr;}
const full_shell::FullShellFrameGeometry* Scene::geometry() const noexcept {return impl_?&impl_->geometry:nullptr;}
const output::physical_run::Replay* Scene::replay() const noexcept {return impl_?&impl_->reader:nullptr;}
const SceneForecast* Scene::forecast() const noexcept {return impl_?&impl_->budget:nullptr;}
std::shared_ptr<const chrono::ChVisualShapeTriangleMesh> Scene::moving_shape() const noexcept {
    return impl_?impl_->shape:nullptr;
}
double Scene::plastic_strain_maximum() const noexcept {return impl_?impl_->plastic_maximum:0;}
} // namespace crash::visual::physical_run
