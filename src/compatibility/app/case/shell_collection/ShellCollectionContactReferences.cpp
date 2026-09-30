#include "ShellCollectionContactGeometryInternal.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include <algorithm>

namespace crash::cases {
namespace sc=tlfea::contact;
namespace fe=tl::fea;
using Code=ShellContactGeometryStatus;
void ShellCollectionContactGeometry::Impl::PrepareCoordinates() {
    coordinates.resize(3*node_count());
    for(std::size_t i=0;i<node_count();++i) {
        const auto x=mapping?mapping->domain()->nodes()[i].position:binding.nodes()[i].position;
        coordinates[3*i]=x.x;coordinates[3*i+1]=x.y;coordinates[3*i+2]=x.z;
    }
    // Only surface geometry determines wall coverage. Beam reference nodes or
    // other valid extra-domain coordinates are not extra contact vertices.
    for(std::size_t i=0;i<binding.node_count();++i) {
        const auto x=binding.nodes()[i].position;
        if(!i)bounds={sc::Vec3{x.x,x.y,x.z},sc::Vec3{x.x,x.y,x.z}};
        auto& lo=bounds[0];auto& hi=bounds[1];
        lo={std::min(lo.x,x.x),std::min(lo.y,x.y),std::min(lo.z,x.z)};
        hi={std::max(hi.x,x.x),std::max(hi.y,x.y),std::max(hi.z,x.z)};
    }
}
ShellContactGeometryReport ShellCollectionContactGeometry::Impl::PrepareReferences(
    const ShellContactGeometryLimits& limits) {
    const auto q=binding.qeph_count(),t=binding.t3_count(),b=binding.qbat_count(),p=q+t+b;
    const sc::VectorView x{coordinates.data(),static_cast<std::uint32_t>(node_count()),3,1};
    std::vector<sc::Q4ParametricReference> quads(q+b);
    std::vector<sc::T3MaterialMeasure> triangles(t);
    std::vector<sc::NodalWallParentInput> inputs(p);
    parents.reserve(p);
    for(std::size_t i=0;i<q+b;++i) {
        const bool qb=i>=q;const auto index=qb?i-q:i;
        const ShellContactParent key{qb?fe::ShellBindingFamily::Qbat:fe::ShellBindingFamily::Qeph,
            index,qb?binding.qbat_source_id(index):binding.qeph_source_id(index)};
        const auto& local=qb?binding.qbat_nodes(index):binding.qeph_nodes(index);
        sc::SurfaceQ4 parent;parent.feature_id=parent.parent_element_id=key.source_id;
        for(unsigned l=0;l<4;++l)parent.nodes[l]=node(local[l]);
        if(!key.source_id||quads[i].Initialize(x,&parent,1).status!=sc::Q4ParametricStatus::Ok)
            return {Code::ReferenceFailure,"Original quadrilateral contact reference cannot be certified",key};
        inputs[i]={&quads[i],0,nullptr};parents.push_back(key);
    }
    for(std::size_t i=0;i<t;++i) {
        const ShellContactParent key{fe::ShellBindingFamily::T3,i,binding.t3_source_id(i)};
        sc::SurfaceTriangle parent;parent.feature_id=parent.parent_element_id=key.source_id;
        parent.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
        for(unsigned l=0;l<3;++l)parent.nodes[l]=node(binding.t3_nodes(i)[l]);
        if(!key.source_id||sc::PrepareT3MaterialMeasure(x,parent,&triangles[i])!=sc::SurfaceMeasureStatus::Ok)
            return {Code::ReferenceFailure,"Original T3 contact reference cannot be certified",key};
        inputs[q+b+i]={nullptr,0,&triangles[i]};parents.push_back(key);
    }
    const auto report=weights.Initialize(static_cast<std::uint32_t>(node_count()),inputs.data(),
        static_cast<std::uint32_t>(p),limits.weights);
    if(report.status!=sc::NodalWallStatus::Ok||weights.node_count()!=binding.node_count()||weights.parent_count()!=p)
        return {Code::WeightFailure,"Contact weights must retain every native parent and shell node"};
    std::sort(parents.begin(),parents.end(),[](auto a,auto b){return a.source_id<b.source_id;});
    for(std::size_t i=0;i<p;++i)
        if(parents[i].source_id!=weights.parent(i).parent_element_id)
            return {Code::WeightFailure,"Contact weight sorting changed native source identity",parents[i]};
    // The checked map is injective. Full shell-node count plus every original
    // parent slot proves coverage; extra domain nodes remain absent from weights.
    return {Code::Ok,"Complete source-mapped shell contact reference prepared"};
}
} // namespace crash::cases
