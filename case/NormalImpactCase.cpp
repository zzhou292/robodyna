#include "NormalImpactCase.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <new>
#include <vector>

namespace crash::case_data {
namespace fea = tl::fea;
namespace contact = tlfea::contact;
namespace {
constexpr std::size_t Capacity = contact::MaxPlanarSurfaceNodes;
bool Positive(double x) { return std::isfinite(x) && x > 0; }
bool FiniteMetrics(const ImpactMetrics& m) {
    for (double value : {m.mass,m.mean_gap,m.mean_normal_velocity,m.kinetic_energy,
                         m.elastic_energy,m.wall_impulse,m.contact_work,m.peak_penetration})
        if (!std::isfinite(value)) return false;
    return Positive(m.mass);
}
using Edge = std::pair<std::uint32_t,std::uint32_t>;

void WallGeometry(const CanonicalWall& source, bool refine,
                  std::vector<contact::PlanarWallVertex>& vertices,
                  std::vector<contact::PlanarWallTriangle>& triangles) {
    for (const auto& v : source.vertices())
        vertices.push_back({{v.position_m[0],v.position_m[1],v.position_m[2]},
                            v.source_node_id,v.assembled_source_node_id});
    for (const auto& t : source.triangles())
        triangles.push_back({{t.vertex_indices[0],t.vertex_indices[1],t.vertex_indices[2]},
                             t.triangle_id,t.source_quad_id,t.assembled_source_quad_id});
    if (!refine) return;
    // Derived midpoint IDs live in an explicit synthetic range. Parent source
    // quad IDs survive; this mesh is never represented as the original deck.
    std::map<Edge,std::uint32_t> midpoints;
    auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
        const Edge edge = std::minmax(a,b);
        auto found = midpoints.find(edge);
        if (found != midpoints.end()) return found->second;
        const auto index = static_cast<std::uint32_t>(vertices.size());
        const auto p = contact::Scale(contact::Add(vertices[a].position,vertices[b].position),.5);
        const std::uint64_t id = (std::uint64_t{1} << 62) + index;
        vertices.push_back({p,id,id}); midpoints.emplace(edge,index); return index;
    };
    std::vector<contact::PlanarWallTriangle> refined;
    for (const auto& t : triangles) {
        const auto a=t.nodes[0], b=t.nodes[1], c=t.nodes[2];
        const auto ab=midpoint(a,b), bc=midpoint(b,c), ca=midpoint(c,a);
        for (const std::array<std::uint32_t,3> v : {std::array<std::uint32_t,3>{a,ab,ca},
                 {ab,b,bc},{ca,bc,c},{ab,bc,ca}})
            refined.push_back({{v[0],v[1],v[2]},static_cast<std::uint64_t>(refined.size()+1),
                               t.source_quad_id,t.assembled_source_quad_id});
    }
    triangles.swap(refined);
}
}  // namespace

struct NormalImpactCase::Impl {
    NormalImpactConfig config;
    fea::FENodalState state;
    contact::PlanarMeshContact contact;
    visual::NodalMeshOutput output;
    ImpactMetrics metrics;
    contact::PlanarContactDiagnostics interval;
    std::array<double,Capacity*3> x{},v{},candidate_x{},candidate_v{};
    std::array<double,Capacity> mass{};
    std::vector<contact::PlanarSurfaceTriangle> triangles;
    double wall_x = 0, triangle_area = 0;
    std::size_t count = 0;
    bool usable = true;

    ImpactMetrics Measure(const std::array<double,Capacity*3>& px,
                          const std::array<double,Capacity*3>& pv,
                          const contact::PlanarContactDiagnostics* applied) const {
        ImpactMetrics next = metrics;
        next.mean_gap=next.mean_normal_velocity=next.kinetic_energy=next.elastic_energy=0;
        for (std::size_t i=0;i<count;++i) {
            next.mean_gap += mass[i]*(wall_x-px[3*i]);
            next.mean_normal_velocity -= mass[i]*pv[3*i];
            next.kinetic_energy += .5*mass[i]*pv[3*i]*pv[3*i];
            bool covered=false;
            if (applied) for (std::size_t t=0;t<triangles.size();++t)
                if (applied->samples[t].covered) for (auto node:triangles[t].local_nodes)
                    covered=covered || node==i;
            if (covered) next.peak_penetration = std::max(next.peak_penetration,px[3*i]-wall_x);
        }
        next.mean_gap/=next.mass; next.mean_normal_velocity/=next.mass;
        if (applied) {
            next.wall_impulse += config.dt*applied->wall_reaction.x;
            for (std::size_t t=0;t<triangles.size();++t) {
                const auto& sample=applied->samples[t];
                double point_x=0, mean_velocity=0;
                for (auto i : triangles[t].local_nodes) {
                    point_x+=px[3*i]/3;
                    mean_velocity+=(v[3*i]+pv[3*i])/6;
                }
                if (sample.covered) {
                    const double depth=std::max(0.0,point_x-wall_x);
                    next.elastic_energy+=.5*sample.stiffness*depth*depth;
                }
                next.contact_work+=config.dt*sample.force_on_surface.x*mean_velocity;
            }
        }
        return next;
    }
};

NormalImpactCase::NormalImpactCase() = default;
NormalImpactCase::~NormalImpactCase() = default;
ImpactReport NormalImpactCase::Initialize(const CanonicalWall& wall, const NormalImpactConfig& config) {
    if (impl_) return {ImpactStatus::AlreadyInitialized,"Impact case already initialized"};
    if (!wall.loaded() || wall.vertices().size()!=62 || wall.triangles().size()!=100 || wall.source_quads().size()!=46 ||
        wall.provenance().model_archive_reference_sha256!="aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451" ||
        !Positive(config.dt) || !Positive(config.areal_density) || !Positive(config.penalty_per_area) ||
        !Positive(config.speed) || !Positive(config.max_penetration) || !Positive(config.width) ||
        !std::isfinite(config.center_y) || !std::isfinite(config.center_z) ||
        (config.patch_divisions!=1 && config.patch_divisions!=2 && config.patch_divisions!=4))
        return {ImpactStatus::InvalidInput,"Invalid canonical wall or bounded normal-impact configuration"};
    try {
        auto s=std::make_unique<Impl>(); s->config=config; s->wall_x=wall.vertices()[0].position_m[0];
        const auto divisions=config.patch_divisions, side=divisions+1;
        s->count=side*side; s->triangle_area=.5*config.width*config.width/(divisions*divisions);
        const double triangle_mass=config.areal_density*s->triangle_area;
        if (!Positive(triangle_mass)) return {ImpactStatus::InvalidInput,"Unrepresentable patch mass"};
        std::vector<contact::PlanarSurfaceNode> nodes;
        for (unsigned z=0;z<side;++z) for (unsigned y=0;y<side;++y) {
            const auto i=z*side+y;
            s->x[3*i]=s->wall_x;
            s->x[3*i+1]=config.center_y+config.width*(double(y)/divisions-.5);
            s->x[3*i+2]=config.center_z+config.width*(double(z)/divisions-.5);
            s->v[3*i]=config.speed;
            nodes.push_back({i,{s->x[3*i],s->x[3*i+1],s->x[3*i+2]},i+1});
        }
        for (unsigned z=0;z<divisions;++z) for (unsigned y=0;y<divisions;++y) {
            const auto a=z*side+y, b=a+1, c=a+side, d=c+1;
            for (const std::array<std::uint32_t,3> t : {std::array<std::uint32_t,3>{a,b,d},{a,d,c}}) {
                s->triangles.push_back({{t[0],t[1],t[2]},s->triangles.size()+1});
                for (auto i:t) s->mass[i]+=triangle_mass/3;
            }
        }
        std::array<double,Capacity> inverse{}; std::array<std::uint8_t,Capacity> fixed{};
        for (std::size_t i=0;i<s->count;++i) { inverse[i]=1/s->mass[i]; s->metrics.mass+=s->mass[i]; }
        if (!Positive(s->metrics.mass)) return {ImpactStatus::InvalidInput,"Unrepresentable total patch mass"};
        s->metrics=s->Measure(s->x,s->v,nullptr);
        if (!FiniteMetrics(s->metrics)) return {ImpactStatus::InvalidInput,"Initial impact ledger arithmetic overflow"};
        fea::NodalStateConfig state_config; state_config.node_count=s->count; state_config.fixed_dt=config.dt;
        auto state_report=s->state.Initialize(state_config,{s->x.data(),s->v.data(),nullptr,s->count},inverse.data(),fixed.data());
        if (state_report.status!=fea::NodalStatus::Ok) return {ImpactStatus::StateFailure,state_report.message};
        std::vector<contact::PlanarWallVertex> vertices;
        std::vector<contact::PlanarWallTriangle> faces;
        WallGeometry(wall,config.refine_wall,vertices,faces);
        contact::PlanarContactConfig contact_config;
        contact_config.owner_id=s->state.accepted().owner_id;
        contact_config.global_node_count=static_cast<std::uint32_t>(s->count);
        contact_config.penalty_per_area=config.penalty_per_area;
        contact_config.max_penetration_m=config.max_penetration;
        auto report=s->contact.Initialize(contact_config,{vertices.data(),static_cast<std::uint32_t>(vertices.size()),
                    faces.data(),static_cast<std::uint32_t>(faces.size())},
                    {nodes.data(),static_cast<std::uint32_t>(nodes.size()),s->triangles.data(),static_cast<std::uint32_t>(s->triangles.size())});
        if (report.status!=contact::PlanarContactStatus::Ok) return {ImpactStatus::ContactFailure,report.message};
        visual::Binding binding; binding.identity={s->state.accepted().owner_id,1,1}; binding.tl_node_count=s->count;
        for (const auto& n:nodes) binding.vertices.push_back({n.global_node,{2,1,n.source_node_id}});
        for (const auto& t:s->triangles)
            binding.triangles.push_back({{t.local_nodes[0],t.local_nodes[1],t.local_nodes[2]},2,1,t.triangle_id,1,0,0});
        auto output_report=s->output.Initialize(s->state,binding);
        if (output_report.status!=visual::Status::Ok) return {ImpactStatus::InvalidOutput,output_report.message};
        output_report=s->output.Publish(s->state);
        if (output_report.status!=visual::Status::Ok) return {ImpactStatus::InvalidOutput,output_report.message};
        s->metrics.stamp=s->state.accepted();
        impl_=std::move(s); return {ImpactStatus::Ok,"Normal mass-patch impact initialized"};
    } catch (const std::bad_alloc&) { return {ImpactStatus::InvalidInput,"Impact startup allocation failed"}; }
}

ImpactReport NormalImpactCase::Step() {
    if (!impl_) return {ImpactStatus::NotInitialized,"Impact case is not initialized"};
    auto& s=*impl_;
    if (!s.usable) return {ImpactStatus::ReadbackFailure,"Impact case stopped after device readback failure"};
    auto reject=[&](ImpactStatus status,const char* message) { s.state.Discard(); return ImpactReport{status,message}; };
    fea::NodalTrialToken token; fea::NodalAssemblyView assembly;
    auto state_report=s.state.BeginTrial(&token,&assembly);
    if (state_report.status!=fea::NodalStatus::Ok) return reject(ImpactStatus::StateFailure,state_report.message);
    auto contact_report=s.contact.Evaluate(assembly);
    if (contact_report.status!=contact::PlanarContactStatus::Ok) return reject(ImpactStatus::ContactFailure,contact_report.message);
    state_report=s.state.SealAssembly(token);
    if (state_report.status!=fea::NodalStatus::Ok) return reject(ImpactStatus::StateFailure,state_report.message);
    state_report=fea::AdvanceTranslations(s.state,token);
    if (state_report.status!=fea::NodalStatus::Ok) return reject(ImpactStatus::StateFailure,state_report.message);
    fea::NodalPreparedView prepared;
    state_report=s.state.BorrowPrepared(token,&prepared);
    if (state_report.status!=fea::NodalStatus::Ok) return reject(ImpactStatus::StateFailure,state_report.message);
    contact_report=s.contact.ValidatePrepared(prepared);
    if (contact_report.status!=contact::PlanarContactStatus::Ok) return reject(ImpactStatus::ContactFailure,contact_report.message);
    const auto* diagnostic=s.contact.diagnostics();
    if (!diagnostic || diagnostic->owner_id!=prepared.owner_id || diagnostic->base_epoch!=s.metrics.stamp.epoch ||
        diagnostic->attempt!=prepared.attempt || diagnostic->sample_count!=s.triangles.size())
        return reject(ImpactStatus::InvalidOutput,"Contact diagnostics do not identify the prepared interval");
    // Read back before commit: a failed output/ledger preparation cannot leave
    // the accepted clock ahead of this case's bookkeeping. No borrowed pointer
    // survives this call. Device errors stop this case; no context recovery claim.
    const auto bytes=3*s.count*sizeof(double);
    auto error=cudaMemcpyAsync(s.candidate_x.data(),prepared.kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,prepared.stream);
    if (error==cudaSuccess) error=cudaMemcpyAsync(s.candidate_v.data(),prepared.kinematics.velocity_xyz,bytes,cudaMemcpyDeviceToHost,prepared.stream);
    if (error==cudaSuccess) error=cudaStreamSynchronize(prepared.stream);
    if (error!=cudaSuccess) { s.usable=false; return reject(ImpactStatus::ReadbackFailure,cudaGetErrorString(error)); }
    const auto interval=*diagnostic;
    auto metrics=s.Measure(s.candidate_x,s.candidate_v,&interval);
    if (!FiniteMetrics(metrics)) return reject(ImpactStatus::InvalidOutput,"Impact ledger arithmetic overflow");
    state_report=s.state.Commit(token);
    if (state_report.status!=fea::NodalStatus::Ok) return reject(ImpactStatus::StateFailure,state_report.message);
    // No-throw publication after the sole accepted-state commit.
    metrics.stamp=s.state.accepted(); s.metrics=metrics; s.interval=interval;
    s.x=s.candidate_x; s.v=s.candidate_v;
    return {ImpactStatus::Ok,"Normal contact interval committed"};
}
visual::Report NormalImpactCase::Publish() {
    return impl_ ? impl_->output.Publish(impl_->state) : visual::Report{visual::Status::NotInitialized,"Impact case is not initialized"};
}
const ImpactMetrics* NormalImpactCase::metrics() const noexcept { return impl_ ? &impl_->metrics : nullptr; }
const contact::PlanarContactDiagnostics* NormalImpactCase::last_interval() const noexcept {
    return impl_ && impl_->interval.valid ? &impl_->interval : nullptr;
}
const visual::NodalMeshOutput* NormalImpactCase::output() const noexcept { return impl_ ? &impl_->output : nullptr; }
fea::NodalAllocationInfo NormalImpactCase::state_allocations() const noexcept {
    return impl_ ? impl_->state.allocations() : fea::NodalAllocationInfo{};
}
contact::PlanarContactAllocationInfo NormalImpactCase::contact_allocations() const noexcept {
    return impl_ ? impl_->contact.allocations() : contact::PlanarContactAllocationInfo{};
}
}  // namespace crash::case_data
