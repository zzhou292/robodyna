#include "WallTessellationGeometry.h"
#include "output/ArtifactIO.h"
#include "collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::case_data::wall_tessellation_detail {
namespace ct=tlfea::contact;
using output::Require;
Edge Key(std::uint32_t a,std::uint32_t b) { return std::minmax(a,b); }
Edges CountEdges(ct::PlanarWallView mesh) {
    Edges result;
    for(unsigned i=0;i<mesh.triangle_count;++i)for(unsigned n=0;n<3;++n)
        ++result[Key(mesh.triangles[i].nodes[n],mesh.triangles[i].nodes[(n+1)%3])];
    return result;
}
std::set<Edge> Boundary(const Edges& edges) {
    std::set<Edge> result;
    for(const auto& e:edges)if(e.second==1)result.insert(e.first);
    return result;
}
bool ConvexCycle(ct::PlanarWallView mesh,std::uint32_t a,std::uint32_t b,double tolerance,std::array<std::uint32_t,4>& cycle) {
    std::map<Edge,unsigned> counts; std::set<unsigned> nodes; std::map<unsigned,unsigned> next;
    for(auto index:{a,b})for(unsigned n=0;n<3;++n) {
        const auto& t=mesh.triangles[index]; nodes.insert(t.nodes[n]); ++counts[Key(t.nodes[n],t.nodes[(n+1)%3])];
    }
    if(nodes.size()!=4||counts.size()!=5)return false;
    for(auto index:{a,b})for(unsigned n=0;n<3;++n) {
        const auto& t=mesh.triangles[index];
        if(counts.at(Key(t.nodes[n],t.nodes[(n+1)%3]))==1&&!next.emplace(t.nodes[n],t.nodes[(n+1)%3]).second)return false;
    }
    if(next.size()!=4)return false;
    std::array<std::uint32_t,4> value{}; auto current=*nodes.begin();
    for(unsigned n=0;n<4;++n) {value[n]=current;const auto it=next.find(current);if(it==next.end())return false;current=it->second;}
    if(current!=value[0]||std::set<unsigned>(value.begin(),value.end()).size()!=4)return false;
    // Resolve convexity well outside subtraction/product rounding. A source
    // coordinate scale enters the bound; near-collinear turns stay unchanged.
    for(unsigned n=0;n<4;++n) {
        const auto p=mesh.vertices[value[n]].position,q=mesh.vertices[value[(n+1)%4]].position,r=mesh.vertices[value[(n+2)%4]].position;
        const auto u=ct::Subtract(q,p),v=ct::Subtract(r,q); const auto cross=ct::geometry_detail::Cross(u,v);
        const double scale=std::max({1.,std::abs(p.y),std::abs(p.z),std::abs(q.y),std::abs(q.z),std::abs(r.y),std::abs(r.z)});
        const double budget=16*tolerance*scale;
        if(!ct::IsFinite(cross)||!std::isfinite(budget)||cross.x>=-budget)return false;
    }
    cycle=value;return true;
}
namespace {
double CoordinateMidpoint(double a,double b,ct::Q4IntegralInterval& truth,double& error) {
    namespace bounds=ct::q4_bounds;
    ct::Q4IntegralInterval first,second;
    Require(bounds::Scale({a,a},.5,&first)&&bounds::Scale({b,b},.5,&second)&&bounds::Add(first,second,&truth),
            "Wall midpoint arithmetic cannot be enclosed");
    const double value=a==b?a:.5*a+.5*b;
    double lower=0,upper=0;
    Require(std::isfinite(value)&&value>=std::min(a,b)&&value<=std::max(a,b)&&
            bounds::AbsoluteDifferenceUpper(value,truth.lower,&lower)&&bounds::AbsoluteDifferenceUpper(value,truth.upper,&upper),
            "Wall midpoint is unresolved on its source segment");
    error=std::max(lower,upper);return value;
}
void AppendInteger(std::string& bytes,std::uint64_t value) {
    // Explicit big-endian bytes; no compiler layout, host endian or locale.
    for(int shift=56;shift>=0;shift-=8)bytes.push_back(static_cast<char>((value>>shift)&255));
}
} // namespace
ct::Vec3 Midpoint(ct::Vec3 a,ct::Vec3 b,WallMidpointSource& meta) {
    Require(a.x==b.x&&ct::IsFinite(a)&&ct::IsFinite(b),"Wall midpoint source plane is invalid");
    meta.coordinate_error={};
    return {a.x,CoordinateMidpoint(a.y,b.y,meta.exact_midpoint_y,meta.coordinate_error.y),
                CoordinateMidpoint(a.z,b.z,meta.exact_midpoint_z,meta.coordinate_error.z)};
}
void CheckBoundary(ct::PlanarWallView original,ct::PlanarWallView derived,WallTessellationMetadata& meta,
                   const std::map<Edge,std::uint32_t>& midpoint,double tolerance) {
    const auto original_edges=Boundary(CountEdges(original)),derived_edges=Boundary(CountEdges(derived));
    meta.original_boundary_edges=original_edges.size();meta.derived_boundary_edges=derived_edges.size();
    std::set<Edge> expected;
    if(meta.kind!=WallTessellationKind::UniformFour)expected=original_edges;
    else for(const auto& edge:original_edges) {
        const auto found=midpoint.find(edge);Require(found!=midpoint.end(),"Missing shared exposed wall midpoint");
        expected.insert(Key(edge.first,found->second));expected.insert(Key(found->second,edge.second));
    }
    Require(expected==derived_edges,"Wall tessellation changed exposed topology");
    for(const auto& vertex:meta.midpoints)if(vertex.exposed_edge) {
        const auto a=original.vertices[vertex.original_edge[0]].position,b=original.vertices[vertex.original_edge[1]].position;
        const auto p=derived.vertices[vertex.vertex].position;
        const bool axis=a.y==b.y||a.z==b.z;
        if(axis)Require((a.y==b.y?p.y==a.y:p.z==a.z),"Axis-aligned exposed wall edge moved");
        double deviation=0;
        if(!axis)Require(ct::q4_bounds::AddScalar(vertex.coordinate_error.y,vertex.coordinate_error.z,true,&deviation),
                         "Exposed wall displacement bound overflow");
        meta.exposed_boundary_displacement_bound_m=std::max(meta.exposed_boundary_displacement_bound_m,deviation);
        meta.exposed_boundary_exact=meta.exposed_boundary_exact&&deviation==0;
    }
    Require(meta.exposed_boundary_displacement_bound_m<=tolerance,
            "Rounded wall boundary exceeds the original geometry roundoff band");
    for(unsigned i=0;i<derived.vertex_count;++i)Require(derived.vertices[i].position.x==original.vertices[0].position.x,
                                                      "Wall tessellation changed the exact plane");
}
std::string MeshSha256(ct::PlanarWallView mesh,WallTessellationKind kind) {
    std::string bytes="robo_dyna.wall_tessellation_mesh.v1";
    AppendInteger(bytes,static_cast<std::uint8_t>(kind)); AppendInteger(bytes,mesh.vertex_count); AppendInteger(bytes,mesh.triangle_count);
    for(unsigned i=0;i<mesh.vertex_count;++i) {
        const auto& v=mesh.vertices[i];
        AppendInteger(bytes,output::Bits(v.position.x));AppendInteger(bytes,output::Bits(v.position.y));AppendInteger(bytes,output::Bits(v.position.z));
        AppendInteger(bytes,v.source_node_id);AppendInteger(bytes,v.assembled_source_node_id);
    }
    for(unsigned i=0;i<mesh.triangle_count;++i) {
        const auto& t=mesh.triangles[i];for(auto node:t.nodes)AppendInteger(bytes,node);
        AppendInteger(bytes,t.triangle_id);AppendInteger(bytes,t.source_quad_id);AppendInteger(bytes,t.assembled_source_quad_id);
    }
    return output::Sha256(bytes);
}
} // namespace crash::case_data::wall_tessellation_detail
