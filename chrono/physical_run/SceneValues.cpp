#include "SceneState.h"
#include "chrono/ReplayDisplayGeometry.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace crash::visual::physical_run {
SceneForecast Forecast(std::size_t replay,std::size_t nodes,std::size_t parents,
        std::size_t triangles,std::size_t points,SceneOptions options) {
    output::Require(options.host_bytes && options.host_bytes<=2048ull<<20,"Physical scene cap exceeds two GiB");
    full_shell::FrameGeometryOptions geometry;
    geometry.geometry=ReplayGeometryLimits::Vehicle();
    geometry.colors=options.colors;
    geometry.parent_activity=true;
    const auto bytes=full_shell::FrameGeometryBudget(nodes,parents,triangles,geometry);
    tl::util::BoundedArenaLayout budget(options.host_bytes),sample(options.host_bytes);
    tl::util::ArenaRegion region;
    // ReadFrame's raw bytes, decoded values and typed validation, then activity.
    output::Require(sample.Append<double>(nodes,region) && sample.Append<double>(nodes,region) &&
        sample.Append<double>(nodes,region) && sample.Append<double>(points,region),"Scene sample exceeds cap");
    const auto values=sample.bytes();
    output::Require(sample.Append<std::byte>(values,region) && sample.Append<std::byte>(values,region) &&
        sample.Append<std::uint8_t>(parents,region) && sample.Append<std::byte>(4u<<20,region),
        "Scene sample/readback workspace exceeds cap");
    output::Require(budget.Append<std::byte>(replay,region) && budget.Append<std::byte>(bytes,region) &&
        budget.Append<std::byte>(sample.bytes(),region) && budget.Append<std::byte>(4u<<20,region),
        "Physical replay/presentation complete host forecast exceeds cap");
    return {replay,bytes,sample.bytes(),budget.bytes()};
}
ScanValues Scan(const output::physical_run::Replay& replay) {
    ScanValues values;
    values.low.fill(std::numeric_limits<double>::infinity());
    values.high.fill(-std::numeric_limits<double>::infinity());
    const auto position=[&](unsigned axis,double x) {
        output::Require(std::isfinite(x),"Physical display position is nonfinite");
        values.low[axis]=std::min(values.low[axis],x);
        values.high[axis]=std::max(values.high[axis],x);
    };
    for(std::size_t i=0;i<replay.index().frames.size();++i) {
        const auto sample=replay.ReadSample(i);
        for(std::size_t n=0;n<sample.frame.position_xyz.size();++n) position(n%3,sample.frame.position_xyz[n]);
        for(double p:sample.frame.plastic_points) values.plastic_maximum=std::max(values.plastic_maximum,p);
    }
    if(const auto wall=replay.wall_mesh())
        for(const auto& v:wall->GetCoordsVertices()) for(unsigned a=0;a<3;++a) position(a,v[a]);
    if(replay.context().points() && values.plastic_maximum==0) values.plastic_maximum=1;
    return values;
}
} // namespace crash::visual::physical_run
