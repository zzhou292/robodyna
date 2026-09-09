#include "WallTessellation.h"
#include "WallTessellationGeometry.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <new>
#include <stdexcept>

namespace crash::case_data {
namespace ct=tlfea::contact;
namespace detail=wall_tessellation_detail;
using Status=WallTessellationStatus;
using output::Require;
const char* WallTessellationName(WallTessellationKind kind) noexcept {
    switch(kind) {
        case WallTessellationKind::Original:return "original";
        case WallTessellationKind::FlipConvexPairs:return "flip-convex-source-pairs";
        case WallTessellationKind::UniformFour:return "uniform-four-per-original-triangle";
    }
    return "invalid";
}
struct WallTessellation::Data {
    std::vector<ct::PlanarWallVertex> vertices;
    std::vector<ct::PlanarWallTriangle> triangles;
    WallTessellationMetadata metadata;
    WallProvenance source;
    ct::PlanarWallGeometry geometry;
    ct::PlanarWallView view() const noexcept {return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
                                                   triangles.data(),static_cast<std::uint32_t>(triangles.size())};}
};
namespace {
struct SourceCopy {
    std::vector<ct::PlanarWallVertex> vertices;
    std::vector<ct::PlanarWallTriangle> triangles;
    explicit SourceCopy(const CanonicalWall& wall) {
        for(const auto& v:wall.vertices()) {
            Require(v.source_node_id<kWallMidpointIdBase&&v.assembled_source_node_id<kWallMidpointIdBase,
                    "Original wall node collides with the synthetic feature namespace");
            vertices.push_back({{v.position_m[0],v.position_m[1],v.position_m[2]},v.source_node_id,v.assembled_source_node_id});
        }
        for(const auto& t:wall.triangles()) {
            Require(t.triangle_id<kWallMidpointIdBase,"Original wall face collides with the synthetic feature namespace");
            triangles.push_back({{t.vertex_indices[0],t.vertex_indices[1],t.vertex_indices[2]},t.triangle_id,t.source_quad_id,t.assembled_source_quad_id});
        }
    }
    ct::PlanarWallView view() const {return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
                                            triangles.data(),static_cast<std::uint32_t>(triangles.size())};}
};
std::uint64_t DerivedFace(WallTessellationKind kind,unsigned ordinal) {
    return kWallDerivedFaceIdBase|(static_cast<std::uint64_t>(kind)<<48)|static_cast<std::uint64_t>(ordinal+1);
}
void Flip(const SourceCopy& source,const CanonicalWall& canonical,double tolerance,
          std::vector<ct::PlanarWallTriangle>& triangles,WallTessellationMetadata& metadata) {
    std::map<std::uint64_t,std::vector<unsigned>> groups;
    std::set<std::uint64_t> stitched;
    for(const auto& entry:canonical.stitching())stitched.insert(entry.source_quad_id);
    for(unsigned t=0;t<source.triangles.size();++t)groups[source.triangles[t].source_quad_id].push_back(t);
    for(const auto& group:groups) {
        if(group.second.size()!=2||stitched.count(group.first))continue;
        const auto a=group.second[0],b=group.second[1];std::array<std::uint32_t,4> q{};
        if(!detail::ConvexCycle(source.view(),a,b,tolerance,q))continue;
        // Determine which of the two physical diagonals is currently present;
        // selecting the opposite pair must actually change connectivity.
        unsigned common=0; std::array<std::uint32_t,2> old{};
        for(auto n:source.triangles[a].nodes)
            if(std::find(std::begin(source.triangles[b].nodes),std::end(source.triangles[b].nodes),n)!=std::end(source.triangles[b].nodes))old[common++]=n;
        Require(common==2,"Convex source pair has invalid common diagonal");
        if(detail::Key(old[0],old[1])==detail::Key(q[0],q[2]))std::rotate(q.begin(),q.begin()+1,q.end());
        else Require(detail::Key(old[0],old[1])==detail::Key(q[1],q[3]),"Convex source pair diagonal is not internal");
        auto& first=triangles[a];auto& second=triangles[b];
        first.nodes[0]=q[0];first.nodes[1]=q[1];first.nodes[2]=q[2];
        second.nodes[0]=q[0];second.nodes[1]=q[2];second.nodes[2]=q[3];
        first.triangle_id=DerivedFace(metadata.kind,a);second.triangle_id=DerivedFace(metadata.kind,b);
        const std::array<std::uint64_t,2> originals{{source.triangles[a].triangle_id,source.triangles[b].triangle_id}};
        metadata.faces[a]={originals,2,0,true};metadata.faces[b]={originals,2,1,true};
        metadata.flipped_source_quads.push_back(group.first);
    }
    Require(!metadata.flipped_source_quads.empty(),"No resolved convex source pair was eligible for diagonal qualification");
}
void Subdivide(const SourceCopy& source,std::vector<ct::PlanarWallVertex>& vertices,
               std::vector<ct::PlanarWallTriangle>& triangles,WallTessellationMetadata& metadata,
               std::map<detail::Edge,std::uint32_t>& midpoints) {
    const auto edges=detail::CountEdges(source.view());
    Require(vertices.size()+edges.size()<=ct::MaxPlanarWallVertices&&source.triangles.size()<=ct::MaxPlanarWallTriangles/4,
            "Uniform wall subdivision exceeds the admitted vertex/triangle capacity");
    for(const auto& edge:edges) {
        const auto a=edge.first.first,b=edge.first.second; WallMidpointSource record;
        record.vertex=vertices.size();record.original_edge={a,b};record.exposed_edge=edge.second==1;
        record.original_source_nodes={vertices[a].source_node_id,vertices[b].source_node_id};
        record.original_assembled_nodes={vertices[a].assembled_source_node_id,vertices[b].assembled_source_node_id};
        const auto point=detail::Midpoint(vertices[a].position,vertices[b].position,record);
        const auto id=kWallMidpointIdBase|static_cast<std::uint64_t>(metadata.midpoints.size()+1);
        vertices.push_back({point,id,id});midpoints.emplace(edge.first,record.vertex);metadata.midpoints.push_back(record);
    }
    triangles.clear();metadata.faces.clear();triangles.reserve(4*source.triangles.size());metadata.faces.reserve(4*source.triangles.size());
    for(const auto& t:source.triangles) {
        const auto a=t.nodes[0],b=t.nodes[1],c=t.nodes[2];
        const auto ab=midpoints.at(detail::Key(a,b)),bc=midpoints.at(detail::Key(b,c)),ca=midpoints.at(detail::Key(c,a));
        const std::array<std::array<std::uint32_t,3>,4> children{{{{a,ab,ca}},{{ab,b,bc}},{{ca,bc,c}},{{ab,bc,ca}}}};
        for(unsigned child=0;child<children.size();++child) {
            const auto& n=children[child];
            triangles.push_back({{n[0],n[1],n[2]},DerivedFace(metadata.kind,triangles.size()),t.source_quad_id,t.assembled_source_quad_id});
            metadata.faces.push_back({{t.triangle_id,0},1,child,true});
        }
    }
}
} // namespace
WallTessellation::WallTessellation()=default;
WallTessellation::~WallTessellation()=default;
WallTessellationReport WallTessellation::Initialize(const CanonicalWall& wall,const std::string& bytes,WallTessellationKind kind) {
    if(data_)return {Status::AlreadyInitialized,"Wall tessellation is already initialized"};
    if(kind!=WallTessellationKind::Original&&kind!=WallTessellationKind::FlipConvexPairs&&kind!=WallTessellationKind::UniformFour)
        return {Status::InvalidInput,"Unknown wall tessellation qualification variant"};
    try {
        CheckCanonicalWallBinding(wall,bytes);SourceCopy original(wall);
        ct::PlanarWallGeometry source_geometry;
        const auto source_report=source_geometry.Initialize(original.view());
        if(source_report.status!=ct::PlanarContactStatus::Ok)return {Status::GeometryFailure,source_report.message};
        auto next=std::make_unique<Data>();next->vertices=original.vertices;next->triangles=original.triangles;next->source=wall.provenance();
        auto& meta=next->metadata;meta.kind=kind;meta.transform_version="robo_dyna.wall_tessellation.v1";
        meta.source_manifest_sha256=kCanonicalWallManifestSha256;meta.original_vertices=original.vertices.size();meta.original_triangles=original.triangles.size();
        for(const auto& t:original.triangles)meta.faces.push_back({{t.triangle_id,0},1,0,false});
        std::map<detail::Edge,std::uint32_t> midpoints;
        if(kind==WallTessellationKind::FlipConvexPairs)Flip(original,wall,source_geometry.tolerance(),next->triangles,meta);
        if(kind==WallTessellationKind::UniformFour)Subdivide(original,next->vertices,next->triangles,meta,midpoints);
        detail::CheckBoundary(original.view(),next->view(),meta,midpoints,source_geometry.tolerance());
        const auto report=next->geometry.Initialize(next->view());
        if(report.status!=ct::PlanarContactStatus::Ok)return {Status::GeometryFailure,report.message};
        meta.mesh_sha256=detail::MeshSha256(next->view(),kind);
        WallTessellationReport success{Status::Ok,"Deterministic wall geometry validated; mechanical coverage/admission remain explicit"};
        data_=std::move(next);return success;
    } catch(const std::bad_alloc&) {return {Status::ResourceLimit,"Wall tessellation host allocation failed"};}
      catch(const std::exception& e) {return {Status::InvalidInput,e.what()};}
}
bool WallTessellation::initialized() const noexcept {return bool(data_);}
ct::PlanarWallView WallTessellation::view() const noexcept {return data_?data_->view():ct::PlanarWallView{};}
const WallTessellationMetadata* WallTessellation::metadata() const noexcept {return data_?&data_->metadata:nullptr;}
const WallProvenance* WallTessellation::source_provenance() const noexcept {return data_?&data_->source:nullptr;}
const ct::PlanarWallGeometry* WallTessellation::geometry() const noexcept {return data_?&data_->geometry:nullptr;}
WallTessellationReport WallTessellation::CheckCoverage(const ct::Q4SurfaceView& reference,const ct::Q4FixedYZMassView& mass,
                                                     double clearance,ct::Q4PlanarGeometry& output) const {
    if(!data_)return {Status::NotInitialized,"Wall tessellation is not initialized"};
    ct::Q4PlanarGeometry next;const auto report=next.Initialize(data_->geometry,reference,mass,clearance);
    if(report.status!=ct::PlanarContactStatus::Ok)return {Status::GeometryFailure,report.message};
    for(unsigned p=0;p<next.view().parent_count;++p)if(!next.view().parents[p].covered)
        return {Status::GeometryFailure,"Original physical Q4 footprint is not wholly covered by derived finite wall"};
    WallTessellationReport success{Status::Ok,"Original fixed Q4 footprint remains covered by validated derived wall"};
    output=next;return success;
}
} // namespace crash::case_data
