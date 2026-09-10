#include "FullShellVisualizationPlan.h"
#include <cmath>
#include <set>

namespace crash::output::full_shell {
namespace {
void Add(std::size_t& sum,std::size_t n,std::size_t cap) {
    Require(sum<=cap&&n<=cap-sum,"Visualization forecast exceeds byte/file capacity");sum+=n;
}
std::size_t Product(std::uint64_t n,std::size_t bytes,std::size_t cap) {
    Require(!bytes||n<=cap/bytes,"Visualization forecast multiplication exceeds capacity");return static_cast<std::size_t>(n)*bytes;
}
}
Plan PlanArchive(const PlanRequest& r) {
    Require(r.total_byte_cap&&r.total_byte_cap<=TotalByteCap&&r.file_byte_cap>=IntervalCoreBytes&&r.file_byte_cap<=kArtifactFileCap&&
        r.static_byte_reserve&&r.static_byte_reserve<=r.total_byte_cap&&r.nodes&&r.nodes<=1048576&&
        r.parents&&r.parents<=1048576&&r.plastic_points<=4194304&&
        r.frames>=2&&r.frames<kArtifactFrameCap&&r.intervals>=r.frames-1&&r.intervals<=UINT64_MAX/2,
        "Invalid visualization archive capacities");
    Require(std::isfinite(r.fixed_dt)&&r.fixed_dt>0&&std::isfinite(r.requested_duration)&&r.requested_duration>0,
        "Invalid visualization horizon/timestep");
    // Count is supplied by the case plan. Check that it is the first mathematical
    // fixed-dt endpoint at/after the requested duration. Actual saved times still
    // come from the accepted owner and are never reconstructed by this planner.
    const long double h=r.fixed_dt,target=r.requested_duration;
    Require(std::isfinite(r.fixed_dt*static_cast<double>(r.intervals))&&
        std::isfinite(h*r.intervals)&&h*r.intervals>=target&&h*(r.intervals-1)<target,
        "Interval count does not match the actual planned timestep/horizon");
    Plan p;const arrays::Limits limits{r.file_byte_cap,UINT32_MAX,64};
    p.position_bytes=arrays::ByteCount({arrays::Scalar::Float64,r.nodes,3,{}},limits);
    p.plastic_bytes=arrays::ByteCount({arrays::Scalar::Float64,r.plastic_points,1,{}},limits);
    p.frame_capacity=r.frames+1;
    p.frame_bytes=p.position_bytes;Add(p.frame_bytes,p.plastic_bytes,r.total_byte_cap);
    Add(p.frame_bytes,r.extra_frame_bytes,r.total_byte_cap);
    Require(r.extra_frame_bytes<=r.file_byte_cap&&r.extra_interval_bytes<=r.file_byte_cap-IntervalCoreBytes,
        "Optional visualization record exceeds file capacity");
    p.interval_row_bytes=IntervalCoreBytes+r.extra_interval_bytes;
    p.rows_per_chunk=r.file_byte_cap/p.interval_row_bytes;
    Require(p.rows_per_chunk,"Interval record cannot fit in a file");
    p.interval_chunks=static_cast<std::size_t>(r.intervals/p.rows_per_chunk+(r.intervals%p.rows_per_chunk!=0));
    Require(p.interval_chunks<=64,"Visualization interval segment capacity exceeded");
    p.interval_bytes=Product(r.intervals,p.interval_row_bytes,r.total_byte_cap);
    Require(!r.static_files.empty()&&r.static_files.size()<=kArtifactInventoryCap,"Missing/oversized static reservation");
    std::set<std::string> names;bool manifest=false,index=false,configuration=false;
    for(const auto& file:r.static_files) {
        arrays::CheckRelativeName(file.file);
        Require(file.bytes&&file.bytes<=r.file_byte_cap&&names.insert(file.file).second,"Invalid static file reservation");
        manifest|=file.file=="manifest.json";index|=file.file=="frame-index.json";configuration|=file.file=="configuration.json";
        Add(p.static_declared_bytes,file.bytes,r.static_byte_reserve);
    }
    Require(manifest&&index&&configuration,"Missing required manifest/index/configuration reservation");
    // Frame JSON metadata is part of the explicit reserve, not an omitted cost.
    Require(Product(p.frame_capacity,FrameMetadataByteCap,r.static_byte_reserve)<=
        r.static_byte_reserve-p.static_declared_bytes,"Frame metadata exceeds remaining static reserve");
    p.forecast_bytes=r.static_byte_reserve;
    Add(p.forecast_bytes,Product(p.frame_capacity,p.frame_bytes,r.total_byte_cap),r.total_byte_cap);
    Add(p.forecast_bytes,p.interval_bytes,r.total_byte_cap);
    // Two typed interval column files per chunk; optional channels may consume
    // at most one extra file per frame/interval chunk in this first profile.
    p.forecast_files=r.static_files.size();
    Add(p.forecast_files,p.frame_capacity*(3+(r.extra_frame_bytes!=0)),kArtifactInventoryCap);
    Add(p.forecast_files,p.interval_chunks*(2+(r.extra_interval_bytes!=0)),kArtifactInventoryCap);
    const std::uint64_t denominator=r.frames-1,q=r.intervals/denominator,rem=r.intervals%denominator;
    for(std::size_t i=0;i<r.frames;++i)p.frame_epochs.push_back(q*i+(rem*i)/denominator);
    return p;
}
} // namespace crash::output::full_shell
