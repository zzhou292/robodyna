#include "AcceptedReplaySourceAssembly.h"
#include "case/CanonicalWallArtifacts.h"
#include <algorithm>
#include <sstream>
namespace crash::output::replay_detail {
namespace {
std::string OriginalMeshIdentity(const case_data::CanonicalWall& wall) {
    // Decode the persisted wall_tessellation_mesh.v1 identity contract from
    // its original source records. This keeps replay independent of TL runtime
    // geometry preparation and distinguishes this digest from the OBJ hash.
    std::string bytes="robo_dyna.wall_tessellation_mesh.v1";
    const auto word=[&](std::uint64_t value){for(int shift=56;shift>=0;shift-=8)bytes.push_back(char((value>>shift)&255));};
    word(0);word(wall.vertices().size());word(wall.triangles().size());
    for(const auto& v:wall.vertices()){for(double x:v.position_m)word(Bits(x));word(v.source_node_id);word(v.assembled_source_node_id);}
    for(const auto& t:wall.triangles()){for(auto n:t.vertex_indices)word(n);word(t.triangle_id);word(t.source_quad_id);word(t.assembled_source_quad_id);}
    return Sha256(bytes);
}
}
void CheckPlacedWall(Bundle& b,const Value& setup,const std::array<double,3>& minimum,const std::array<double,3>& maximum) {
    const auto bytes=VerifiedBytes(b,"original-canonical-wall.manifest.json");std::istringstream input(bytes);case_data::CanonicalWall original;
    const auto report=original.Load(input);Require(report.status==case_data::WallStatus::Ok,report.message);case_data::CheckCanonicalWallBinding(original,bytes);
    const auto p=Json(VerifiedBytes(b,"placed-wall-placement.json"));const auto mesh=ReadMesh(b,"placed-wall.mesh.json");
    Require(Text(p,"schema")=="robo_dyna.placed_canonical_wall.v1"&&Text(p,"source_manifest_sha256")==case_data::kCanonicalWallManifestSha256&&
        Text(p,"source_mesh_sha256")==OriginalMeshIdentity(original)&&Text(p,"placed_mesh_sha256")==b.inventory.at("placed-wall.mesh.json").hash&&
        Text(p,"placed_obj_sha256")==b.inventory.at("placed-wall.obj").hash,"Placed wall source/mesh hash association failed");
    Require(original.vertices().size()==62&&original.triangles().size()==100&&original.source_quads().size()==46&&
        mesh->GetCoordsVertices().size()==62&&mesh->GetIndicesVertices().size()==100&&Unsigned(p,"vertex_count")==62&&Unsigned(p,"triangle_count")==100,
        "Placed wall omitted original canonical geometry");
    const double shift=Real(p,"declared_translation_x_m"),wall_x=Real(p,"represented_wall_x_m");
    Require(Bits(shift)==Unsigned(p,"declared_translation_x_binary64")&&Bits(wall_x)==Unsigned(p,"represented_wall_x_binary64")&&
        Bits(wall_x)==Bits(.05+shift),"Placed wall transform bits or represented X disagree");
    const auto& vertices=WallArray(p,"vertex_identity",62);const auto& triangles=WallArray(p,"triangle_identity",100);
    for(unsigned n=0;n<62;++n) {
        const auto& ids=vertices[n];const auto& source=original.vertices()[n];Require(ids.IsArray()&&ids.Size()==3,"Placed wall vertex identity shape mismatch");
        for(const auto& id:ids.GetArray())Require(id.IsUint64(),"Placed wall vertex ID type mismatch");
        Require(ids[0].GetUint64()==n&&ids[1].GetUint64()==source.source_node_id&&ids[2].GetUint64()==source.assembled_source_node_id,
            "Placed wall vertex identity changed");
        const auto& actual=mesh->GetCoordsVertices()[n];Require(Bits(actual.x())==Bits(wall_x)&&Bits(actual.y())==Bits(source.position_m[1])&&
            Bits(actual.z())==Bits(source.position_m[2]),"Placed wall contains an undeclared coordinate change");
    }
    for(unsigned t=0;t<100;++t) {
        const auto& ids=triangles[t];const auto& source=original.triangles()[t];Require(ids.IsArray()&&ids.Size()==7,"Placed wall triangle identity shape mismatch");
        for(const auto& id:ids.GetArray())Require(id.IsUint64(),"Placed wall triangle ID type mismatch");
        Require(ids[0].GetUint64()==t&&ids[1].GetUint64()==source.triangle_id&&ids[2].GetUint64()==source.source_quad_id&&
            ids[3].GetUint64()==source.assembled_source_quad_id,"Placed wall source triangle identity changed");
        for(unsigned j=0;j<3;++j)Require(ids[4+j].GetUint64()==source.vertex_indices[j]&&
            mesh->GetIndicesVertices()[t][j]==int(source.vertex_indices[j]),"Placed wall source connectivity changed");
        b.wall_faces.push_back(source.triangle_id);
    }
    Require(Bits(shift)==Bits((maximum[0]+Real(setup,"declared_leading_gap_m"))-.05),"Placed wall does not implement the declared original-source leading gap");
    const auto& gap=WallNumbers(setup,"actual_leading_gap_interval_m",2);const long double actual_gap=static_cast<long double>(wall_x)-maximum[0];
    Require(gap[0].GetDouble()>0&&gap[0].GetDouble()<=actual_gap&&gap[1].GetDouble()>=actual_gap,"Placed wall actual gap certificate changed");
    const auto& low=WallNumbers(setup,"projected_motion_minimum_m",3);const auto& high=WallNumbers(setup,"projected_motion_maximum_m",3);
    const auto& qlow=WallNumbers(setup,"query_minimum_m",3);const auto& qhigh=WallNumbers(setup,"query_maximum_m",3);
    Require(Bits(low[0].GetDouble())==Bits(wall_x)&&Bits(high[0].GetDouble())==Bits(wall_x)&&
        Bits(qlow[0].GetDouble())==Bits(wall_x)&&Bits(qhigh[0].GetDouble())==Bits(wall_x),"Coverage is not projected at the actual placed wall X");
    const auto bounds=original.bounds_m();const double margin=Real(setup,"motion_margin_m");
    for(unsigned j=1;j<3;++j)Require(low[j].GetDouble()<=static_cast<long double>(minimum[j])-margin&&
        high[j].GetDouble()>=static_cast<long double>(maximum[j])+margin&&qlow[j].GetDouble()<=low[j].GetDouble()&&
        qhigh[j].GetDouble()>=high[j].GetDouble()&&qlow[j].GetDouble()>=bounds[0][j]&&qhigh[j].GetDouble()<=bounds[1][j],
        "Archived projected source envelope or finite wall bounds disagree");
    b.wall_x=wall_x;
    if(b.assembly)b.assembly->wall=mesh;
}
void CheckPlacedSourcePartWall(Bundle& b,const Document& configuration) {
    const auto& setup=Member(configuration,"wall_setup");const auto& nodes=WallArray(configuration,"reference_nodes",117);
    std::array<double,3> minimum{},maximum{};const auto& first=WallNumbers(nodes[0],"reference_xyz_m",3);
    for(unsigned j=0;j<3;++j)minimum[j]=maximum[j]=first[j].GetDouble();
    for(const auto& node:nodes.GetArray()) {const auto& x=WallNumbers(node,"reference_xyz_m",3);
        for(unsigned j=0;j<3;++j){minimum[j]=std::min(minimum[j],x[j].GetDouble());maximum[j]=std::max(maximum[j],x[j].GetDouble());}}
    CheckPlacedWall(b,setup,minimum,maximum);
}

}
