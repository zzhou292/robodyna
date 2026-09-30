#include "PlacedCanonicalWall.h"
#include "output/ArtifactIO.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace crash::case_data {
using namespace output;
void WritePlacedCanonicalWallArtifacts(const std::filesystem::path& directory,const PlacedCanonicalWall& wall) {
    Require(wall.initialized()&&wall.placement()&&wall.source_manifest(),"Placed wall output requires immutable preparation");
    const auto view=wall.view();chrono::ChTriangleMeshConnected mesh;
    for(std::uint32_t n=0;n<view.vertex_count;++n) {
        const auto x=view.vertices[n].position;mesh.GetCoordsVertices().emplace_back(x.x,x.y,x.z);
    }
    for(std::uint32_t p=0;p<view.triangle_count;++p) {
        const auto& t=view.triangles[p];mesh.GetIndicesVertices().emplace_back(t.nodes[0],t.nodes[1],t.nodes[2]);
    }
    WriteBytes(directory/"original-canonical-wall.manifest.json",*wall.source_manifest());
    WriteMeshFiles(directory,"placed-wall",mesh);
    Document d;d.SetObject();String(d,"schema","robo_dyna.placed_canonical_wall.v1");
    String(d,"source_manifest_sha256",wall.placement()->source_manifest_sha256);
    String(d,"source_mesh_sha256",wall.placement()->source_mesh_sha256);
    String(d,"transform","X = original_X + declared_translation_x; Y and Z unchanged");
    Number(d,"declared_translation_x_m",wall.placement()->translation_x_m);
    Integer(d,"declared_translation_x_binary64",Bits(wall.placement()->translation_x_m));
    Number(d,"represented_wall_x_m",wall.placement()->wall_x_m);
    Integer(d,"represented_wall_x_binary64",Bits(wall.placement()->wall_x_m));
    String(d,"placed_mesh_sha256",Sha256(ReadBounded(directory/"placed-wall.mesh.json",1024*1024)));
    String(d,"placed_obj_sha256",Sha256(ReadBounded(directory/"placed-wall.obj",1024*1024)));
    Integer(d,"vertex_count",view.vertex_count);Integer(d,"triangle_count",view.triangle_count);
    Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType);
    for(std::uint32_t n=0;n<view.vertex_count;++n) {
        const auto& v=view.vertices[n];Value row(rapidjson::kArrayType);
        for(std::uint64_t id:{std::uint64_t(n),v.source_node_id,v.assembled_source_node_id})row.PushBack(Value().SetUint64(id),d.GetAllocator());
        vertices.PushBack(row,d.GetAllocator());
    }
    for(std::uint32_t p=0;p<view.triangle_count;++p) {
        const auto& t=view.triangles[p];Value row(rapidjson::kArrayType);
        for(std::uint64_t id:{std::uint64_t(p),t.triangle_id,t.source_quad_id,t.assembled_source_quad_id,
                             std::uint64_t(t.nodes[0]),std::uint64_t(t.nodes[1]),std::uint64_t(t.nodes[2])})
            row.PushBack(Value().SetUint64(id),d.GetAllocator());
        triangles.PushBack(row,d.GetAllocator());
    }
    String(d,"vertex_identity_columns","vertex,source_node,assembled_source_node");
    String(d,"triangle_identity_columns","triangle,triangle_id,source_quad,assembled_source_quad,v0,v1,v2");
    d.AddMember("vertex_identity",vertices,d.GetAllocator());d.AddMember("triangle_identity",triangles,d.GetAllocator());
    WriteJson(directory/"placed-wall-placement.json",d);
}
} // namespace crash::case_data
