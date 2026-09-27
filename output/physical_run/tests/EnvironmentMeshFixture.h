#pragma once
#include "Support.h"
#include "../EnvironmentArtifacts.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
namespace crash::output::physical_run::test {
// Record-format fixture only. No canonical source/physical admission is inferred
// from these values; the actual owning test uses the immutable envelope source.
struct DeclaredMesh {
    ft::Directory directory;
    records::Context context=Context();
    records::source::CanonicalData source;
    EnvironmentReceipt receipt;
    Document document;
    DeclaredMesh() {
        source.inputs.canonical_manifest.sha256=Sha256("vehicle canonical fixture");
        source.inputs.scope_report.sha256=Sha256("vehicle scope fixture");
        source.inputs.source_member.sha256=Sha256("vehicle member fixture");
        receipt.source_instance_id=context.identity().source_instance;
        receipt.wall_binding_id=91;receipt.part_id=301;
        receipt.source_mapping_sha256=context.identity().source_mapping_sha256;
        chrono::ChTriangleMeshConnected mesh;
        mesh.GetCoordsVertices()={{2.,-1.,-1.},{2.,1.,-1.},{2.,1.,1.},{2.,-1.,1.}};
        mesh.GetIndicesVertices()={{0,1,2},{0,2,3}};
        WriteMeshFiles(directory.path,"environment-wall",mesh);
        for(unsigned i=0;i<2;++i) {
            const auto bytes=ReadBounded(directory.path/EnvironmentFiles[i],EnvironmentFileCap);
            receipt.files[i]={EnvironmentFiles[i],Sha256(bytes),bytes.size()};
        }
        document.SetObject();
        String(document,"schema","robo_dyna.native_environment_wall.v1");String(document,"profile",EnvironmentProfile);
        String(document,"purpose","declared_fixed_physical_geometry_not_canonical_wall_or_restart");
        Integer(document,"source_instance_id",receipt.source_instance_id);Integer(document,"wall_binding_id",receipt.wall_binding_id);
        Integer(document,"part_id",receipt.part_id);Integer(document,"element_id",401);
        Integer(document,"material_id",501);Integer(document,"section_id",601);
        String(document,"source_mapping_sha256",receipt.source_mapping_sha256);
        String(document,"vehicle_canonical_sha256",source.inputs.canonical_manifest.sha256);
        String(document,"vehicle_scope_sha256",source.inputs.scope_report.sha256);
        String(document,"vehicle_source_member_sha256",source.inputs.source_member.sha256);
        String(document,"wall_source_sha256",Sha256("actual wall source fixture"));
        String(document,"namespace_sha256",Sha256("namespace fixture"));
        String(document,"mesh_sha256",receipt.files[0].sha256);String(document,"obj_sha256",receipt.files[1].sha256);
        Integer(document,"vehicle_physical_nodes",9);Integer(document,"physical_nodes",13);
        Integer(document,"vehicle_render_nodes",context.nodes());Integer(document,"vehicle_parents",context.parents().size());
        Integer(document,"physical_parents",context.parents().size()+1);
        Value ids(rapidjson::kArrayType),nodes(rapidjson::kArrayType),points(rapidjson::kArrayType);
        Value triangles(rapidjson::kArrayType),bounds(rapidjson::kArrayType);
        for(unsigned i=0;i<4;++i) {
            ids.PushBack(Value().SetUint64(1001+i),document.GetAllocator());
            nodes.PushBack(Value().SetUint64(9+i),document.GetAllocator());
            Value row(rapidjson::kArrayType);
            for(unsigned k=0;k<3;++k)row.PushBack(Value().SetDouble(mesh.GetCoordsVertices()[i][k]),document.GetAllocator());
            points.PushBack(row,document.GetAllocator());
        }
        for(const auto& face:mesh.GetIndicesVertices()) {
            Value row(rapidjson::kArrayType);
            for(unsigned k=0;k<3;++k)row.PushBack(Value().SetUint(face[k]),document.GetAllocator());
            triangles.PushBack(row,document.GetAllocator());
        }
        for(double edge:{-.5,.5}) {
            Value row(rapidjson::kArrayType);
            for(unsigned k=0;k<3;++k)row.PushBack(Value().SetDouble(edge),document.GetAllocator());
            bounds.PushBack(row,document.GetAllocator());
        }
        document.AddMember("node_ids",ids,document.GetAllocator());document.AddMember("domain_nodes",nodes,document.GetAllocator());
        document.AddMember("reference_m",points,document.GetAllocator());document.AddMember("triangles",triangles,document.GetAllocator());
        document.AddMember("vehicle_reference_bounds_m",bounds,document.GetAllocator());
        Number(document,"young_pa",200e9);Number(document,"poisson",.3);Number(document,"density_kg_m3",7860);
        Number(document,"thickness_m",.001);Number(document,"friction",.6);
        Number(document,"contact_front_plane_m",2.-.0005);Number(document,"reference_plane_m",2.);
        Number(document,"reference_offset_m",.0005);Number(document,"native_half_gap",.5);
        Number(document,"native_working_length_m",.001);Number(document,"wall_mass_kg",31.44);
        receipt.files[2]=WriteDocument(directory.path,EnvironmentFiles[2],document,EnvironmentFileCap);
    }
    auto Read() const {return ReadEnvironmentArtifacts(directory.path,receipt,source,context);}
    void Replace(Document&& replacement,unsigned ordinal) {
        const auto staged=WriteDocument(directory.path,"changed-"+std::to_string(ordinal)+".json",replacement,EnvironmentFileCap);
        const auto bytes=ReadFile(directory.path,staged,EnvironmentFileCap);
        records::test::Overwrite(directory.path/EnvironmentFiles[2],bytes);
        receipt.files[2]={EnvironmentFiles[2],Sha256(bytes),bytes.size()};
    }
};
} // namespace crash::output::physical_run::test
