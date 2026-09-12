#pragma once
#include "Scene.h"
#include "chrono/physics/ChSystemNSC.h"
namespace crash::visual::physical_run {
struct ScanValues {
    std::array<double,3> low,high;
    double plastic_maximum=0;
};
ScanValues Scan(const SampleSource&);
struct Scene::Impl {
    explicit Impl(const SampleSource& r):reader(r) {}
    SampleSource reader;
    full_shell::FullShellFrameGeometry geometry;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> shape;
    ReplayCamera camera;
    ReplayStamp stamp;
    SceneForecast budget;
    double plastic_maximum=0;
};
} // namespace crash::visual::physical_run
