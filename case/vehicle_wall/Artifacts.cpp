#include "Artifacts.h"
#include "output/ArtifactIO.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
namespace crash::cases::vehicle_wall {
using namespace output;
void WriteSetupArtifacts(const std::filesystem::path& directory,const VehicleWallSetup& setup) {
    Require(std::filesystem::is_directory(directory) && std::filesystem::is_empty(directory),
        "Wall setup artifacts require an empty caller-owned directory");
    case_data::WritePlacedCanonicalWallArtifacts(directory,setup.wall());
    const auto view=setup.selected_wall_view();
    chrono::ChTriangleMeshConnected mesh;
    for (std::size_t i=0;i<view.vertex_count;++i) {
        const auto& x=view.vertices[i].position;
        mesh.GetCoordsVertices().emplace_back(x.x,x.y,x.z);
    }
    for (std::size_t i=0;i<view.triangle_count;++i) {
        const auto& t=view.triangles[i];
        mesh.GetIndicesVertices().emplace_back(t.nodes[0],t.nodes[1],t.nodes[2]);
    }
    WriteMeshFiles(directory,"selected-wall",mesh);
    Document document;
    document.SetObject();
    String(document,"schema","robo_dyna.vehicle_wall_setup.v1");
    const auto& settings=setup.settings();
    String(document,"mesh_profile",settings.mesh_profile==WallMeshProfile::PlacedOriginal ?
        "placed-original" : "envelope-rectangle-v1");
    String(document,"scope","immutable setup and coverage; no loaded-step admission");
    String(document,"original_manifest_sha256",setup.wall().placement()->source_manifest_sha256);
    String(document,"selected_mesh_sha256",Sha256(ReadBounded(directory/"selected-wall.mesh.json",1u<<20)));
    String(document,"selected_obj_sha256",Sha256(ReadBounded(directory/"selected-wall.obj",1u<<20)));
    const auto& resolution=setup.execution().resolution();
    const auto& source=resolution.source();
    const auto& canonical=source.canonical().data();
    String(document,"vehicle_archive_sha256",canonical.archive_sha256);
    String(document,"vehicle_canonical_sha256",canonical.inputs.canonical_manifest.sha256);
    String(document,"vehicle_source_member_sha256",canonical.inputs.source_member.sha256);
    String(document,"vehicle_scope_sha256",canonical.inputs.scope_report.sha256);
    String(document,"vehicle_declaration_sha256",source.identity().sha256);
    String(document,"vehicle_resolution_sha256",resolution.identity().sha256);
    String(document,"vehicle_resolution_profile",modelio::vehicle::OriginalRigidProfileName);
    String(document,"retained_shell_ids_sha256",canonical.retained_shell_ids_sha256);
    Integer(document,"wall_binding_id",settings.wall_binding_id);
    Number(document,"speed_mps",settings.initial_speed_mps);
    Number(document,"requested_duration_s",settings.requested_duration_s);
    Number(document,"requested_leading_gap_m",settings.leading_gap_m);
    Number(document,"transverse_margin_m",settings.transverse_margin_m);
    Number(document,"exposed_clearance_m",settings.exposed_clearance_m);
    Number(document,"stiffness_per_area",settings.stiffness_per_area);
    Number(document,"maximum_penetration_m",settings.maximum_penetration_m);
    Number(document,"parent_force_error",settings.parent_force_error);
    Number(document,"parent_energy_error",settings.parent_energy_error);
    Number(document,"represented_wall_x_m",setup.placement().represented_wall_x_m);
    String(document,"original_coverage",setup.original_coverage_report().message);
    String(document,"selected_coverage",setup.coverage_report().message);
    Integer(document,"shell_parents",setup.geometry().weights()->parent_count());
    Integer(document,"surface_nodes",setup.geometry().weights()->node_count());
    Integer(document,"physical_nodes",setup.geometry().weights()->global_node_count());
    Integer(document,"host_peak_upper_bound",setup.forecast().peak_host_upper_bound);
    Value boxes(rapidjson::kArrayType);
    for (auto v : {setup.placement().declared_world_envelope.minimum,setup.placement().declared_world_envelope.maximum}) {
        Value row(rapidjson::kArrayType);
        for (double value : {v.x,v.y,v.z}) row.PushBack(Value().SetDouble(value),document.GetAllocator());
        boxes.PushBack(row,document.GetAllocator());
    }
    document.AddMember("declared_world_envelope_m",boxes,document.GetAllocator());
    Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType);
    for (std::size_t i=0;i<view.vertex_count;++i) {
        Value row(rapidjson::kArrayType);
        for (std::uint64_t value : {std::uint64_t(i),view.vertices[i].source_node_id,view.vertices[i].assembled_source_node_id})
            row.PushBack(Value().SetUint64(value),document.GetAllocator());
        vertices.PushBack(row,document.GetAllocator());
    }
    for (std::size_t i=0;i<view.triangle_count;++i) {
        const auto& t=view.triangles[i];
        Value row(rapidjson::kArrayType);
        for (std::uint64_t value : {std::uint64_t(i),t.triangle_id,t.source_quad_id,t.assembled_source_quad_id})
            row.PushBack(Value().SetUint64(value),document.GetAllocator());
        triangles.PushBack(row,document.GetAllocator());
    }
    document.AddMember("vertex_feature_ids",vertices,document.GetAllocator());
    document.AddMember("triangle_feature_ids",triangles,document.GetAllocator());
    WriteJson(directory/"vehicle-wall-setup.json",document);
}
} // namespace crash::cases::vehicle_wall
