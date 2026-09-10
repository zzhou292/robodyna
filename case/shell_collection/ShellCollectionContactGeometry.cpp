#include "ShellCollectionContactGeometry.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include <algorithm>
#include <new>
#include <vector>

namespace crash::cases {
namespace sc=tlfea::contact;
namespace fe=tl::fea;
using Code=ShellContactGeometryStatus;
struct ShellCollectionContactGeometry::Impl {
    explicit Impl(const fe::ShellBatchBinding& b):binding(b) {}
    fe::ShellBatchBinding binding;
    std::vector<double> coordinates;
    std::vector<ShellContactParent> parents;
    sc::NodalWallWeights weights;
    std::array<sc::Vec3,2> bounds{};
    std::size_t startup_bytes=0;
};
ShellCollectionContactGeometry::ShellCollectionContactGeometry()=default;
ShellCollectionContactGeometry::~ShellCollectionContactGeometry()=default;
ShellContactGeometryReport ShellCollectionContactGeometry::Initialize(
    const fe::ShellBatchBinding& binding,const ShellContactGeometryLimits& limits) {
    if(impl_||!binding.prepared()) return {Code::InvalidInput,"Fresh geometry and a complete native binding are required"};
    const auto q=binding.qeph_count(),t=binding.t3_count(),n=binding.node_count(),p=q+t;
    const bool vehicle=limits.weights.profile==sc::NodalWallWeightProfile::Vehicle;
    if(!vehicle&&limits.weights.profile!=sc::NodalWallWeightProfile::Legacy)
        return {Code::InvalidInput,"Unknown shell contact weight profile"};
    const auto maximum_nodes=vehicle?sc::MaxVehicleWallWeightNodes:sc::MaxNodalWallWeightNodes;
    const auto maximum_parents=vehicle?sc::MaxVehicleWallWeightParents:sc::MaxNodalWallWeightParents;
    const std::size_t maximum_startup=vehicle?ShellContactGeometryLimits::Vehicle().max_startup_bytes:32*1024*1024;
    if(!n||n>maximum_nodes||!p||p>maximum_parents||
       n>limits.weights.max_nodes||p>limits.weights.max_parents||
       (vehicle&&(limits.weights.max_nodes>maximum_nodes||limits.weights.max_parents>maximum_parents))||
       !limits.max_startup_bytes||limits.max_startup_bytes>maximum_startup)
        return {Code::ResourceLimit,"Shell contact startup exceeds explicit collection limits"};
    // Counts passed profile hard bounds, so this sum cannot overflow. Keep the
    // conservative legacy ledger; Vehicle also reserves transient weight indexes.
    // Charge the complete allowed weights payload, including its inline arrays.
    const auto maximum_weights=vehicle?sc::MaxVehicleWallWeightOwnedBytes:4*1024*1024;
    if(!limits.weights.max_owned_bytes||limits.weights.max_owned_bytes>maximum_weights||
       (vehicle&&(!limits.weights.max_startup_bytes||limits.weights.max_startup_bytes>sc::MaxVehicleWallWeightScratchBytes)))
        return {Code::ResourceLimit,"Invalid immutable contact weight payload budget"};
    const auto payload=sizeof(Impl)+binding.host_bytes()+3*n*sizeof(double)+p*sizeof(ShellContactParent)+
        q*sizeof(sc::Q4ParametricReference)+t*sizeof(sc::T3MaterialMeasure)+
        p*sizeof(sc::NodalWallParentInput)+limits.weights.max_owned_bytes+
        (vehicle?limits.weights.max_startup_bytes:0);
    if(payload>limits.max_startup_bytes)
        return {Code::ResourceLimit,"Shell contact peak startup payload exceeds its byte budget"};
    try {
        auto next=std::make_unique<Impl>(binding); next->startup_bytes=payload;
        next->coordinates.resize(3*n); next->parents.reserve(p);
        for(std::size_t i=0;i<n;++i) {
            const auto x=binding.nodes()[i].position;
            next->coordinates[3*i]=x.x; next->coordinates[3*i+1]=x.y; next->coordinates[3*i+2]=x.z;
            if(!i) next->bounds={sc::Vec3{x.x,x.y,x.z},sc::Vec3{x.x,x.y,x.z}};
            auto& lo=next->bounds[0];auto& hi=next->bounds[1];
            lo={std::min(lo.x,x.x),std::min(lo.y,x.y),std::min(lo.z,x.z)};
            hi={std::max(hi.x,x.x),std::max(hi.y,x.y),std::max(hi.z,x.z)};
        }
        const sc::VectorView x{next->coordinates.data(),static_cast<std::uint32_t>(n),3,1};
        std::vector<sc::Q4ParametricReference> quads(q);
        std::vector<sc::T3MaterialMeasure> triangles(t);
        std::vector<sc::NodalWallParentInput> inputs(p);
        for(std::size_t i=0;i<q;++i) {
            const ShellContactParent key{fe::ShellBindingFamily::Qeph,i,binding.qeph_source_id(i)};
            sc::SurfaceQ4 parent; parent.feature_id=parent.parent_element_id=key.source_id;
            for(unsigned l=0;l<4;++l) parent.nodes[l]=binding.qeph_nodes(i)[l];
            if(!key.source_id||quads[i].Initialize(x,&parent,1).status!=sc::Q4ParametricStatus::Ok)
                return {Code::ReferenceFailure,"Original QEPH contact reference cannot be certified",key};
            inputs[i]={&quads[i],0,nullptr};next->parents.push_back(key);
        }
        for(std::size_t i=0;i<t;++i) {
            const ShellContactParent key{fe::ShellBindingFamily::T3,i,binding.t3_source_id(i)};
            sc::SurfaceTriangle parent; parent.feature_id=parent.parent_element_id=key.source_id;
            parent.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
            for(unsigned l=0;l<3;++l) parent.nodes[l]=binding.t3_nodes(i)[l];
            if(!key.source_id||sc::PrepareT3MaterialMeasure(x,parent,&triangles[i])!=sc::SurfaceMeasureStatus::Ok)
                return {Code::ReferenceFailure,"Original T3 contact reference cannot be certified",key};
            inputs[q+i]={nullptr,0,&triangles[i]};next->parents.push_back(key);
        }
        const auto report=next->weights.Initialize(static_cast<std::uint32_t>(n),inputs.data(),
            static_cast<std::uint32_t>(p),limits.weights);
        if(report.status!=sc::NodalWallStatus::Ok||next->weights.node_count()!=n||next->weights.parent_count()!=p)
            return {Code::WeightFailure,"Contact weights must retain every native parent and node"};
        std::sort(next->parents.begin(),next->parents.end(),[](auto a,auto b){return a.source_id<b.source_id;});
        for(std::size_t i=0;i<p;++i)
            if(next->parents[i].source_id!=next->weights.parent(i).parent_element_id)
                return {Code::WeightFailure,"Contact weight sorting changed native source identity",next->parents[i]};
        for(std::size_t i=0;i<n;++i) if(next->weights.node(i).node!=i)
            return {Code::WeightFailure,"Contact weights omitted or reordered a native physical node"};
        impl_=std::move(next);return {Code::Ok,"Complete shell contact reference prepared"};
    } catch(const std::bad_alloc&) { return {Code::ResourceLimit,"Shell contact startup allocation failed"}; }
}
bool ShellCollectionContactGeometry::prepared() const noexcept { return bool(impl_); }
const fe::ShellBatchBinding* ShellCollectionContactGeometry::binding() const noexcept { return impl_?&impl_->binding:nullptr; }
const sc::NodalWallWeights* ShellCollectionContactGeometry::weights() const noexcept { return impl_?&impl_->weights:nullptr; }
sc::VectorView ShellCollectionContactGeometry::positions() const noexcept {
    return impl_?sc::VectorView{impl_->coordinates.data(),static_cast<std::uint32_t>(impl_->binding.node_count()),3,1}:sc::VectorView{};
}
std::array<sc::Vec3,2> ShellCollectionContactGeometry::reference_bounds() const noexcept {
    return impl_?impl_->bounds:std::array<sc::Vec3,2>{};
}
sc::PlanarContactReport ShellCollectionContactGeometry::CheckWallCoverage(const sc::PlanarWallGeometry& wall,
    sc::PlanarWallBox motion,double clearance,std::uint64_t id,sc::PlanarWallBoxCoverage* output) const {
    using Status=sc::PlanarContactStatus;
    if(!impl_||!wall.initialized())return {Status::NotInitialized,"Shell collection or finite wall is not prepared"};
    const auto lo=motion.minimum,hi=motion.maximum;
    if(!output||!id||!sc::IsFinite(lo)||!sc::IsFinite(hi)||
       lo.x>impl_->bounds[0].x||lo.y>impl_->bounds[0].y||lo.z>impl_->bounds[0].z||
       hi.x<impl_->bounds[1].x||hi.y<impl_->bounds[1].y||hi.z<impl_->bounds[1].z)
        return {Status::InvalidInput,"Motion envelope must contain the complete native shell collection"};
    const double x=wall.wall_x();
    return sc::CheckPlanarWallBox(wall,{{x,lo.y,lo.z},{x,hi.y,hi.z}},clearance,id,
        sc::PlanarWallBoxMode::ConservativeExpansion,output);
}
const ShellContactParent* ShellCollectionContactGeometry::parent_from_weight(std::size_t i) const noexcept {
    return impl_&&i<impl_->parents.size()?&impl_->parents[i]:nullptr;
}
std::size_t ShellCollectionContactGeometry::startup_payload_bytes() const noexcept { return impl_?impl_->startup_bytes:0; }
} // namespace crash::cases
