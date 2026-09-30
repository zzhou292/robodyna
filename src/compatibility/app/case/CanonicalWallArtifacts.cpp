#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <sstream>

namespace crash::case_data {
using namespace crash::output;
void CheckCanonicalWallBinding(const CanonicalWall& wall,const std::string& bytes) {
    Require(wall.loaded()&&Sha256(bytes)==kCanonicalWallManifestSha256,"Artifact input is not the verified canonical wall");
    // Authenticate the actual supplied geometry, not just an unrelated byte
    // string. Reuse the same bounded loader; this is startup-only work.
    CanonicalWall expected;std::istringstream input(bytes);
    Require(expected.Load(input).status==WallStatus::Ok,"Pinned canonical input no longer satisfies its schema");
    const auto& source=wall.provenance();const auto& pinned_source=expected.provenance();
    Require(source.wall_file==pinned_source.wall_file&&source.combine_file==pinned_source.combine_file&&
        source.wall_sha256==pinned_source.wall_sha256&&source.combine_sha256==pinned_source.combine_sha256&&
        source.model_archive_reference_sha256==pinned_source.model_archive_reference_sha256&&
        source.generator_sha256==pinned_source.generator_sha256&&source.obj_sha256==pinned_source.obj_sha256,
        "Wall source provenance differs from pinned input");
    Require(wall.vertices().size()==expected.vertices().size()&&wall.triangles().size()==expected.triangles().size()&&
        wall.source_quads().size()==expected.source_quads().size()&&wall.stitching().size()==expected.stitching().size(),"Wall does not match pinned source counts");
    for(std::size_t i=0;i<wall.vertices().size();++i) {
        const auto& a=wall.vertices()[i];const auto& b=expected.vertices()[i];
        Require(a.vertex_index==b.vertex_index&&a.source_node_id==b.source_node_id&&a.assembled_source_node_id==b.assembled_source_node_id,"Wall node source binding differs from pinned input");
        for(unsigned j=0;j<3;++j)Require(Bits(a.position_m[j])==Bits(b.position_m[j]),"Wall coordinates differ from pinned input");
    }
    for(std::size_t i=0;i<wall.triangles().size();++i) {
        const auto& a=wall.triangles()[i];const auto& b=expected.triangles()[i];
        Require(a.vertex_indices==b.vertex_indices&&a.source_node_ids==b.source_node_ids&&a.triangle_id==b.triangle_id&&
            a.source_quad_id==b.source_quad_id&&a.assembled_source_quad_id==b.assembled_source_quad_id,"Wall triangle source binding differs from pinned input");
    }
    for(std::size_t i=0;i<wall.source_quads().size();++i) {
        const auto& a=wall.source_quads()[i];const auto& b=expected.source_quads()[i];
        Require(a.source_node_ids==b.source_node_ids&&a.source_quad_id==b.source_quad_id&&a.assembled_source_quad_id==b.assembled_source_quad_id&&
            a.source_part_id==b.source_part_id&&a.assembled_source_part_id==b.assembled_source_part_id,"Wall quad source binding differs from pinned input");
    }
    for(std::size_t i=0;i<wall.stitching().size();++i) {
        const auto& a=wall.stitching()[i];const auto& b=expected.stitching()[i];
        Require(a.source_quad_id==b.source_quad_id&&a.source_edge==b.source_edge&&a.inserted_source_node_ids==b.inserted_source_node_ids,"Wall stitch source binding differs from pinned input");
    }
    Require(wall.reaction_groups().whole_wall_triangle_ids==expected.reaction_groups().whole_wall_triangle_ids&&
        wall.reaction_groups().source_segment_set_1001_triangle_ids==expected.reaction_groups().source_segment_set_1001_triangle_ids,"Wall reaction binding differs from pinned input");
}

std::string ReadPinnedWallManifest(const std::string& path) {
    auto bytes=ReadBounded(path,1024*1024);
    Require(Sha256(bytes)==kCanonicalWallManifestSha256,"Canonical wall manifest SHA256 differs from the required pin");
    return bytes;
}
void WriteCanonicalWallArtifacts(const std::filesystem::path& directory,const CanonicalWall& wall,
                                 const std::string& bytes) {
    CheckCanonicalWallBinding(wall,bytes);
    chrono::ChTriangleMeshConnected mesh;
    for(const auto& v:wall.vertices())mesh.GetCoordsVertices().emplace_back(v.position_m[0],v.position_m[1],v.position_m[2]);
    for(const auto& t:wall.triangles())mesh.GetIndicesVertices().emplace_back(t.vertex_indices[0],t.vertex_indices[1],t.vertex_indices[2]);
    WriteBytes(directory/"canonical-wall.manifest.json",bytes);
    WriteMeshFiles(directory,"canonical-wall",mesh);
}
} // namespace crash::case_data
