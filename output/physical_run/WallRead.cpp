#include "WallArtifacts.h"
#include "output/BoundedArrayJson.h"
#include "output/GeometryMeshArchive.h"
#include "case/CanonicalWallArtifacts.h"
#include <set>
namespace crash::output::physical_run {
namespace {
const Value& Field(const Value& value,const char* name) {
    Require(value.IsObject() && value.HasMember(name),"Missing physical wall field");return value[name];
}
std::string Text(const Value& value,const char* name) {return array_json::Text(Field(value,name));}
void Unique(const Value& value) {
    std::set<std::string> names;
    for(const auto& member:value.GetObject())Require(names.emplace(member.name.GetString(),member.name.GetStringLength()).second,
        "Duplicate physical wall metadata member");
}
void Hash(const Value& value,const char* name,const std::string& expected) {
    Require(Text(value,name)==expected,"Physical wall hash/source association differs");
}
void FeatureRows(const Value& document,const char* name,std::size_t count,unsigned columns) {
    const auto& rows=Field(document,name);
    Require(rows.IsArray() && rows.Size()==count,"Physical wall feature count differs");
    for(std::size_t i=0;i<count;++i) {
        const auto& row=rows[i];Require(row.IsArray() && row.Size()==columns,"Physical wall feature shape differs");
        for(const auto& id:row.GetArray())array_json::UInt(id);
        Require(row[0].GetUint64()==i,"Physical wall feature index differs");
    }
}
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> ReadWallArtifacts(const std::filesystem::path& root,
    const WallReceipt& receipt,const records::source::CanonicalData& source,const records::Context& context) {
    WallDocument(receipt);
    Require(receipt.source_instance_id==context.identity().source_instance &&
        receipt.source_mapping_sha256==context.identity().source_mapping_sha256,"Physical wall belongs to another run source");
    for(const auto& file:receipt.files)ReadFile(root,file,WallFileCap);
    Require(receipt.files[0].sha256==case_data::kCanonicalWallManifestSha256,"Physical wall original source is not pinned");
    const auto setup=array_json::Parse(ReadFile(root,receipt.files[6],WallFileCap),WallFileCap);
    Unique(setup);
    Require(Text(setup,"schema")=="robo_dyna.vehicle_wall_setup.v1" &&
        array_json::UInt(Field(setup,"wall_binding_id"))==receipt.wall_binding_id,"Physical wall setup identity differs");
    Hash(setup,"original_manifest_sha256",receipt.files[0].sha256);
    Hash(setup,"selected_mesh_sha256",receipt.files[4].sha256);Hash(setup,"selected_obj_sha256",receipt.files[5].sha256);
    Hash(setup,"vehicle_archive_sha256",source.archive_sha256);
    Hash(setup,"vehicle_canonical_sha256",source.inputs.canonical_manifest.sha256);
    Hash(setup,"vehicle_source_member_sha256",source.inputs.source_member.sha256);
    Hash(setup,"vehicle_scope_sha256",source.inputs.scope_report.sha256);
    Hash(setup,"retained_shell_ids_sha256",source.retained_shell_ids_sha256);
    Require(array_json::UInt(Field(setup,"shell_parents"))==context.parents().size() &&
        array_json::UInt(Field(setup,"surface_nodes"))==context.nodes(),"Physical wall surface belongs to another source scope");
    const auto placement=array_json::Parse(ReadFile(root,receipt.files[3],WallFileCap),WallFileCap);
    Unique(placement);
    Require(Text(placement,"schema")=="robo_dyna.placed_canonical_wall.v1","Unknown placed wall schema");
    Hash(placement,"source_manifest_sha256",receipt.files[0].sha256);
    Hash(placement,"placed_mesh_sha256",receipt.files[1].sha256);Hash(placement,"placed_obj_sha256",receipt.files[2].sha256);
    const auto placed=ReadGeometryMesh(ReadFile(root,receipt.files[1],WallFileCap));
    Require(placed->GetNumVertices()==62 && placed->GetNumTriangles()==100,"Placed wall omitted original geometry");
    const auto mesh=ReadGeometryMesh(ReadFile(root,receipt.files[4],WallFileCap));
    FeatureRows(setup,"vertex_feature_ids",mesh->GetNumVertices(),3);
    FeatureRows(setup,"triangle_feature_ids",mesh->GetNumTriangles(),4);
    FeatureRows(placement,"vertex_identity",placed->GetNumVertices(),3);
    FeatureRows(placement,"triangle_identity",placed->GetNumTriangles(),7);
    const double wall_x=array_json::Real(Field(setup,"represented_wall_x_m"));
    Require(Bits(wall_x)==Bits(array_json::Real(Field(placement,"represented_wall_x_m"))) &&
        Bits(wall_x)==array_json::UInt(Field(placement,"represented_wall_x_binary64")),"Physical wall plane bits differ");
    for(const auto& vertex:mesh->GetCoordsVertices())Require(Bits(vertex.x())==Bits(wall_x),"Selected mesh leaves recorded wall plane");
    for(const auto& vertex:placed->GetCoordsVertices())Require(Bits(vertex.x())==Bits(wall_x),"Placed mesh leaves recorded wall plane");
    const auto profile=Text(setup,"mesh_profile");
    Require(profile=="placed-original" || profile=="envelope-rectangle-v1","Unknown selected finite wall profile");
    if(profile=="placed-original")Require(receipt.files[1].sha256==receipt.files[4].sha256,
        "Placed-original profile selected a different wall mesh");
    else Require(mesh->GetNumVertices()==4 && mesh->GetNumTriangles()==2,"Envelope rectangle shape differs");
    return mesh;
}
} // namespace crash::output::physical_run
