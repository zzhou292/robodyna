#include "SourceAssemblySurface.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"

namespace crash::output::assembly {
struct SourceAssemblySurface::Impl {
    explicit Impl(const source::SourceAssembly& input):source(input) {}
    source::SourceAssembly source;
    visual::Binding binding;
    std::vector<ParentMapping> parents;
    std::vector<std::uint64_t> triangle_parents;
    std::size_t host_bytes=0;
};
SourceAssemblySurface SourceAssemblySurface::Prepare(const source::SourceAssembly& source,
        visual::Identity identity, std::uint64_t asset, std::uint64_t instance, SurfaceLimits limits) {
    const auto& data=source.data();
    Require(identity.owner&&identity.run&&identity.topology&&asset&&instance,"Missing assembly surface identity");
    Require(limits.max_nodes&&limits.max_nodes<=2048&&limits.max_parents&&limits.max_parents<=1024&&
        limits.max_host_bytes&&limits.max_host_bytes<=4*1024*1024&&data.nodes.size()<=limits.max_nodes&&
        data.parents.size()<=limits.max_parents&&!data.nodes.empty()&&!data.parents.empty(),"Assembly surface capacity exceeded");
    Require(data.qeph_count<=data.parents.size()&&data.t3_count==data.parents.size()-data.qeph_count,
        "Assembly family counts do not cover source parents");
    const auto triangle_count=2*data.qeph_count+data.t3_count; // Counts bounded above before multiplication.
    tl::util::BoundedArenaLayout budget(limits.max_host_bytes);tl::util::ArenaRegion ignored;
    Require(budget.Append<Impl>(1,ignored)&&budget.Append<visual::VertexBinding>(data.nodes.size(),ignored)&&
        budget.Append<visual::TriangleBinding>(triangle_count,ignored)&&budget.Append<ParentMapping>(data.parents.size(),ignored)&&
        budget.Append<std::uint64_t>(triangle_count,ignored),"Assembly surface host-byte capacity exceeded");
    auto next=std::make_shared<Impl>(source);next->host_bytes=budget.bytes();
    auto& b=next->binding;b.identity=identity;b.tl_node_count=data.nodes.size();
    b.vertices.reserve(data.nodes.size());b.triangles.reserve(triangle_count);
    next->parents.reserve(data.parents.size());next->triangle_parents.reserve(triangle_count);
    for(std::size_t n=0;n<data.nodes.size();++n)
        b.vertices.push_back({static_cast<std::uint32_t>(n),{asset,instance,data.nodes[n].source_id}});
    std::size_t q=0,t=0;
    for(std::size_t i=0;i<data.parents.size();++i) {
        const auto& p=data.parents[i];const bool quad=p.family==source::ShellFamily::Qeph;
        Require(p.index==i&&p.arity==(quad?4u:3u)&&p.family_index==(quad?q++:t++),"Invalid source family ordering");
        for(unsigned j=0;j<p.arity;++j)Require(p.nodes[j]<b.vertices.size(),"Invalid source global node mapping");
        next->parents.push_back({p.source_id,p.part_id,p.material_id,p.section_id,p.curve_id,i,p.family_index,
            b.triangles.size(),quad?2u:1u,p.source_elform,p.family});
        const auto a=static_cast<std::uint32_t>(p.nodes[0]),bb=static_cast<std::uint32_t>(p.nodes[1]),
            c=static_cast<std::uint32_t>(p.nodes[2]),d=static_cast<std::uint32_t>(p.nodes[3]);
        b.triangles.push_back({{a,bb,c},asset,instance,p.source_id,p.part_id,0,0});
        next->triangle_parents.push_back(p.source_id);
        if(quad) {
            b.triangles.push_back({{a,c,d},asset,instance,p.source_id,p.part_id,0,1});
            next->triangle_parents.push_back(p.source_id);
        }
    }
    Require(q==data.qeph_count&&t==data.t3_count,"Incomplete source family mapping");
    return SourceAssemblySurface(std::move(next));
}
const source::SourceAssembly& SourceAssemblySurface::source() const noexcept{return impl_->source;}
const visual::Binding& SourceAssemblySurface::binding() const noexcept{return impl_->binding;}
const std::vector<ParentMapping>& SourceAssemblySurface::parents() const noexcept{return impl_->parents;}
const std::vector<std::uint64_t>& SourceAssemblySurface::triangle_parents() const noexcept{return impl_->triangle_parents;}
std::size_t SourceAssemblySurface::host_bytes() const noexcept{return impl_->host_bytes;}
} // namespace crash::output::assembly
