#include "RunState.h"
#include "output/physical_frames/Mapping.h"
#include "case/vehicle_wall/native/EnvelopePhysicalSource.h"
#include "output/MeshArchive.h"
#include "output/BoundedArrayJson.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
namespace crash::output::physical_run {
namespace {
const cases::vehicle_wall::native::EnvelopePhysicalSource& CheckEnvironment(
    const physical_frames::Mapping& mapping,const records::Context& context,Profile profile) {
    const auto* source=mapping.environment();
    Require(source && !profile.native_contact && !profile.self_contact &&
        context.identity().source_mapping_sha256==mapping.source_mapping().digest() &&
        context.identity().source_instance==source->domain().source_instance_id() &&
        context.nodes()==mapping.physical_nodes().size() && context.parents().size()==mapping.parents().size() &&
        profile.beam18==bool(mapping.structural_beams()) && profile.type45,
        "Environment archive requires the actual complete physical mapping/profile");
    Require(source->wall().declaration().profile==cases::vehicle_wall::native::Profile::EnvelopeFixedElasticV1,
        "Environment archive has an unsupported declared source profile");
    return *source;
}
records::RecordFile ExistingFile(const std::filesystem::path& root,const char* name) {
    const auto path=arrays::CheckedPath(root,name,true);
    const auto bytes=std::filesystem::file_size(path);Require(bytes && bytes<=EnvironmentFileCap,"Declared environment file exceeds cap");
    const auto data=ReadBounded(path,bytes);return {name,Sha256(data),static_cast<std::size_t>(bytes)};
}
EnvironmentReceipt WriteEnvironment(const std::filesystem::path& root,const physical_frames::Mapping& mapping,
    const records::Context& context) {
    const auto& source=*mapping.environment();const auto& wall=source.wall();const auto& geometry=wall.geometry();
    const auto& ids=wall.ids();const auto& material=wall.declaration().material;
    for(const auto* name:EnvironmentFiles)arrays::CheckedPath(root,name,false);
    chrono::ChTriangleMeshConnected mesh;
    for(const auto& point:geometry.reference_m)mesh.GetCoordsVertices().emplace_back(point.x,point.y,point.z);
    mesh.GetIndicesVertices()={{0,1,2},{0,2,3}};
    WriteMeshFiles(root,"environment-wall",mesh);
    EnvironmentReceipt receipt;receipt.source_instance_id=source.domain().source_instance_id();
    receipt.wall_binding_id=wall.declaration().binding_id;receipt.part_id=ids.part;
    receipt.source_mapping_sha256=mapping.source_mapping().digest();
    receipt.files[0]=ExistingFile(root,EnvironmentFiles[0]);receipt.files[1]=ExistingFile(root,EnvironmentFiles[1]);
    const auto& canonical=mapping.source_mapping().source().data();
    Document doc;doc.SetObject();String(doc,"schema","robo_dyna.native_environment_wall.v1");String(doc,"profile",EnvironmentProfile);
    String(doc,"purpose","declared_fixed_physical_geometry_not_canonical_wall_or_restart");
    Integer(doc,"source_instance_id",receipt.source_instance_id);Integer(doc,"wall_binding_id",receipt.wall_binding_id);
    Integer(doc,"part_id",ids.part);Integer(doc,"element_id",ids.shell);Integer(doc,"material_id",ids.material);Integer(doc,"section_id",ids.section);
    String(doc,"source_mapping_sha256",receipt.source_mapping_sha256);String(doc,"vehicle_canonical_sha256",canonical.inputs.canonical_manifest.sha256);
    String(doc,"vehicle_scope_sha256",canonical.inputs.scope_report.sha256);String(doc,"vehicle_source_member_sha256",canonical.inputs.source_member.sha256);
    String(doc,"wall_source_sha256",wall.digest());String(doc,"namespace_sha256",wall.namespace_report().digest);
    String(doc,"mesh_sha256",receipt.files[0].sha256);String(doc,"obj_sha256",receipt.files[1].sha256);
    Integer(doc,"vehicle_physical_nodes",source.embedding().original().node_count());Integer(doc,"physical_nodes",source.domain().node_count());
    Integer(doc,"vehicle_render_nodes",context.nodes());Integer(doc,"vehicle_parents",context.parents().size());
    Integer(doc,"physical_parents",mapping.physical().execution()->parents().size());
    Value nodes(rapidjson::kArrayType),domain(rapidjson::kArrayType),points(rapidjson::kArrayType),triangles(rapidjson::kArrayType),bounds(rapidjson::kArrayType);
    for(unsigned i=0;i<4;++i) {
        nodes.PushBack(Value().SetUint64(ids.nodes[i]),doc.GetAllocator());
        domain.PushBack(Value().SetUint64(source.environment_parent().domain_nodes[i]),doc.GetAllocator());
        Value row(rapidjson::kArrayType);const auto& p=geometry.reference_m[i];
        for(double v:{p.x,p.y,p.z})row.PushBack(Value().SetDouble(v),doc.GetAllocator());points.PushBack(row,doc.GetAllocator());
    }
    for(const auto& triangle:mesh.GetIndicesVertices()) {
        Value row(rapidjson::kArrayType);for(unsigned k=0;k<3;++k)row.PushBack(Value().SetUint(triangle[k]),doc.GetAllocator());
        triangles.PushBack(row,doc.GetAllocator());
    }
    for(const auto& box:wall.vehicle_prefix().reference_bounds) {
        Value row(rapidjson::kArrayType);for(double v:{box.x,box.y,box.z})row.PushBack(Value().SetDouble(v),doc.GetAllocator());
        bounds.PushBack(row,doc.GetAllocator());
    }
    doc.AddMember("node_ids",nodes,doc.GetAllocator());doc.AddMember("domain_nodes",domain,doc.GetAllocator());
    doc.AddMember("reference_m",points,doc.GetAllocator());doc.AddMember("triangles",triangles,doc.GetAllocator());
    doc.AddMember("vehicle_reference_bounds_m",bounds,doc.GetAllocator());
    Number(doc,"young_pa",material.young_pa);Number(doc,"poisson",material.poisson);Number(doc,"density_kg_m3",material.density_kg_m3);
    Number(doc,"thickness_m",material.thickness_m);Number(doc,"friction",wall.declaration().wall_friction);
    Number(doc,"contact_front_plane_m",geometry.placement.represented_wall_x_m);Number(doc,"reference_plane_m",geometry.reference_plane_m);
    Number(doc,"reference_offset_m",geometry.reference_offset_m);Number(doc,"native_half_gap",geometry.native_half_gap);
    Number(doc,"native_working_length_m",geometry.native_working_length_m);Number(doc,"wall_mass_kg",geometry.wall_mass_kg);
    receipt.files[2]=WriteDocument(root,EnvironmentFiles[2],doc,EnvironmentFileCap);
    ReadEnvironmentArtifacts(root,receipt,canonical,context);
    return receipt;
}
}
Forecast RunArchive::PreflightWithEnvironment(const physical_frames::Mapping& mapping,const records::Context& context,
    records::source::BundleRequest request,Profile profile,Limits limits) {
    CheckEnvironment(mapping,context,profile);
    auto forecast=PreflightCore(mapping.source_mapping(),context,std::move(request),profile,limits,false,true);
    Require(forecast.peak_host_bytes<=limits.host_bytes && EnvironmentWorkspaceBytes<=limits.host_bytes-forecast.peak_host_bytes,
        "Complete environment archive startup exceeds host cap");
    forecast.peak_host_bytes+=EnvironmentWorkspaceBytes;
    return forecast;
}
RunArchive RunArchive::PrepareWithEnvironment(const std::filesystem::path& root,const physical_frames::Mapping& mapping,
    const records::Context& context,records::source::BundleRequest request,Profile profile,Limits limits) {
    const auto complete=PreflightWithEnvironment(mapping,context,request,profile,limits);
    auto archive=PrepareCore(root,mapping.source_mapping(),context,std::move(request),profile,limits,false,true);
    archive.data_->manifest.environment=WriteEnvironment(root,mapping,context);
    archive.data_->forecast=complete;
    return archive;
}
}
