#include "EnvironmentArtifacts.h"
#include "output/BoundedArrayJson.h"
#include "output/GeometryMeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <set>
namespace crash::output::physical_run {
std::shared_ptr<const chrono::ChTriangleMeshConnected> ReadEnvironmentArtifacts(const std::filesystem::path& root,
    const EnvironmentReceipt& receipt,const records::source::CanonicalData& source,const records::Context& context) {
    using namespace array_json;
    EnvironmentDocument(receipt);
    Require(receipt.source_instance_id==context.identity().source_instance &&
        receipt.source_mapping_sha256==context.identity().source_mapping_sha256,"Environment belongs to another archive source");
    for(const auto& file:receipt.files)ReadFile(root,file,EnvironmentFileCap);
    const auto doc=Parse(ReadFile(root,receipt.files[2],EnvironmentFileCap),EnvironmentFileCap);
    Keys(doc,{"schema","profile","purpose","source_instance_id","wall_binding_id","part_id","element_id","material_id","section_id",
        "source_mapping_sha256","vehicle_canonical_sha256","vehicle_scope_sha256","vehicle_source_member_sha256",
        "wall_source_sha256","namespace_sha256","mesh_sha256","obj_sha256","vehicle_physical_nodes","physical_nodes",
        "vehicle_render_nodes","vehicle_parents","physical_parents","node_ids","domain_nodes","reference_m","triangles",
        "young_pa","poisson","density_kg_m3","thickness_m","friction","contact_front_plane_m","reference_plane_m",
        "reference_offset_m","native_half_gap","native_working_length_m","wall_mass_kg","vehicle_reference_bounds_m"});
    Require(Text(doc["schema"])=="robo_dyna.native_environment_wall.v1" && Text(doc["profile"])==EnvironmentProfile &&
        Text(doc["purpose"])=="declared_fixed_physical_geometry_not_canonical_wall_or_restart",
        "Unknown declared fixed environment schema");
    Require(UInt(doc["source_instance_id"])==receipt.source_instance_id && UInt(doc["wall_binding_id"])==receipt.wall_binding_id &&
        UInt(doc["part_id"])==receipt.part_id && Text(doc["source_mapping_sha256"])==receipt.source_mapping_sha256 &&
        Text(doc["vehicle_canonical_sha256"])==source.inputs.canonical_manifest.sha256 &&
        Text(doc["vehicle_scope_sha256"])==source.inputs.scope_report.sha256 &&
        Text(doc["vehicle_source_member_sha256"])==source.inputs.source_member.sha256,
        "Environment source/vehicle identity differs");
    arrays::CheckHash(Text(doc["wall_source_sha256"]));arrays::CheckHash(Text(doc["namespace_sha256"]));
    Require(Text(doc["mesh_sha256"])==receipt.files[0].sha256 && Text(doc["obj_sha256"])==receipt.files[1].sha256,
        "Declared environment mesh companions differ");
    const auto prefix=UInt(doc["vehicle_physical_nodes"]),complete=UInt(doc["physical_nodes"]);
    Require(prefix && complete<=524288 && prefix+4==complete && prefix>=context.nodes() &&
        UInt(doc["vehicle_render_nodes"])==context.nodes() && UInt(doc["vehicle_parents"])==context.parents().size() &&
        UInt(doc["physical_parents"])==context.parents().size()+1,
        "Declared environment physical prefix/counts differ");
    const auto& ids=doc["node_ids"];const auto& nodes=doc["domain_nodes"];const auto& points=doc["reference_m"];
    Require(ids.IsArray()&&ids.Size()==4&&nodes.IsArray()&&nodes.Size()==4&&points.IsArray()&&points.Size()==4,
        "Declared environment node descriptor differs");
    std::set<std::uint64_t> unique;
    for(unsigned i=0;i<4;++i) {
        Require(UInt(ids[i]) && unique.insert(UInt(ids[i])).second && UInt(nodes[i])==prefix+i &&
            points[i].IsArray()&&points[i].Size()==3,"Declared environment source nodes are not a unique suffix");
        for(unsigned k=0;k<3;++k)(void)Real(points[i][k]);
    }
    for(const char* field:{"element_id","material_id","section_id"})Require(UInt(doc[field])>0,"Environment identity is zero");
    for(const char* field:{"young_pa","density_kg_m3","thickness_m","native_working_length_m","wall_mass_kg"})
        Require(Real(doc[field])>0,"Environment physical declaration must be positive");
    Require(Real(doc["poisson"])>=0 && Real(doc["poisson"])<.5 && Real(doc["friction"])>=0,
        "Invalid declared environment material/interface value");
    const auto front=Real(doc["contact_front_plane_m"]),reference=Real(doc["reference_plane_m"]);
    Require(Real(doc["native_half_gap"])>0 && Real(doc["reference_offset_m"])>0 &&
        Bits(Real(doc["native_half_gap"]))==Bits(.5*(Real(doc["thickness_m"])/Real(doc["native_working_length_m"]))) &&
        Bits(Real(doc["reference_offset_m"]))==Bits(Real(doc["native_half_gap"])*Real(doc["native_working_length_m"])) &&
        Bits(reference)==Bits(front+Real(doc["reference_offset_m"])),"Environment reference/contact-plane association differs");
    const auto& bounds=doc["vehicle_reference_bounds_m"];
    Require(bounds.IsArray()&&bounds.Size()==2&&bounds[0].IsArray()&&bounds[1].IsArray()&&
        bounds[0].Size()==3&&bounds[1].Size()==3,"Vehicle-only bounds are unavailable");
    for(unsigned k=0;k<3;++k)Require(Real(bounds[0][k])<=Real(bounds[1][k]),"Vehicle-only bounds are inverted");
    const unsigned expected[2][3]{{0,1,2},{0,2,3}};const auto& triangles=doc["triangles"];
    Require(triangles.IsArray()&&triangles.Size()==2,"Declared wall triangle count differs");
    for(unsigned i=0;i<2;++i) {
        Require(triangles[i].IsArray()&&triangles[i].Size()==3,"Declared wall triangle shape differs");
        for(unsigned k=0;k<3;++k)Require(UInt(triangles[i][k])==expected[i][k],"Declared wall topology differs");
    }
    const auto mesh=ReadGeometryMesh(ReadFile(root,receipt.files[0],EnvironmentFileCap),4,2);
    Require(mesh->GetNumVertices()==4&&mesh->GetNumTriangles()==2,"Declared wall mesh extent differs");
    for(unsigned i=0;i<4;++i)for(unsigned k=0;k<3;++k)
        Require(Bits(mesh->GetCoordsVertices()[i][k])==Bits(Real(points[i][k])),"Declared wall mesh coordinate bits differ");
    for(unsigned i=0;i<2;++i)for(unsigned k=0;k<3;++k)
        Require(mesh->GetIndicesVertices()[i][k]==int(expected[i][k]),"Declared wall mesh indices differ");
    for(const auto& point:mesh->GetCoordsVertices())Require(Bits(point.x())==Bits(reference),"Declared wall leaves its fixed reference plane");
    return mesh;
}
}
