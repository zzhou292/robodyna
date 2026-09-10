#include "AcceptedReplaySourceAssembly.h"
#include "source_assembly/SourceAssemblyWallSchema.h"

namespace crash::output::replay_detail {
void CheckAssemblyStamp(const Bundle& b,const Entry& e,const Value& s) {
    const auto& a=*b.assembly;
    Require(Unsigned(s,"owner_id")==e.owner&&Unsigned(s,"epoch")==e.epoch&&Unsigned(s,"node_count")==b.info.node_count&&
        Text(s,"temporal_scheme")=="staggered_half_kick_start"&&Text(s,"velocity_phase")== (e.epoch?"previous_midpoint":"collocated"),
        "Assembly accepted stamp changed");
    AssemblyEqual(Real(s,"time"),e.time);AssemblyEqual(Real(s,"fixed_dt"),b.fixed_dt);
    AssemblyEqual(Real(s,"velocity_time"),e.epoch?e.interval_base_time+.5*b.fixed_dt:0);
    AssemblyEqual(Real(s,"reaction_time"),e.interval_base_time);
    AssemblyEqual(Real(s,"reaction_kick_dt"),e.epoch?(e.epoch==1?.5*b.fixed_dt:b.fixed_dt):0);
    Require(Unsigned(s,"reaction_base_epoch")== (e.epoch?e.epoch-1:0),"Assembly reaction base epoch changed");
    WallBool(s,"has_rotations",true);WallBool(s,"reactions_valid",e.epoch!=0);
    const auto& groups=Member(s,"rigid_groups");
    Require(Unsigned(groups,"source_instance_id")==a.instance&&Unsigned(groups,"group_count")==a.group_count&&
        Unsigned(groups,"member_count")==a.member_count,"Assembly accepted rigid-group descriptor changed");
}
void CheckSourceAssemblyFields(const Bundle& b,const Entry& e,const chrono::ChTriangleMeshConnected& mesh) {
    const auto& a=*b.assembly;const auto& s=a.source.data();
    const auto d=Json(VerifiedBytes(b,e.mesh.substr(0,e.mesh.size()-10)+".fields.json"));
    Require(Text(d,"schema")==assembly::WallFrameSchema&&Text(d,"kind")==assembly::WallArtifactKind&&
        Text(d,"source_inventory_sha256")==s.identity.sha256&&Unsigned(d,"source_inventory_bytes")==s.identity.bytes&&
        Unsigned(d,"owner_id")==e.owner&&Unsigned(d,"run_id")==b.info.run_id&&Unsigned(d,"topology_id")==b.info.topology_id&&
        Unsigned(d,"source_instance_id")==a.instance&&Unsigned(d,"configuration_id")==b.source_configuration_id&&
        Unsigned(d,"qualification_id")==b.qualification_id&&Unsigned(d,"wall_binding_id")==b.wall_binding_id&&
        Unsigned(d,"accepted_epoch")==e.epoch&&Text(d,"stress_frame")=="native_corotational_shell_axes",
        "Assembly accepted frame identity or stress frame changed");
    AssemblyEqual(Real(d,"accepted_time_s"),e.time);CheckAssemblyStamp(b,e,Member(d,"stamp"));
    const auto& n=Member(d,"nodal_fields");
    Require(Text(n,"position_phase")=="accepted_endpoint"&&Text(n,"velocity_phase")== (e.epoch?"previous_midpoint":"collocated"),
        "Assembly nodal phase changed");
    const auto& x=WallNumbers(n,"position_xyz_m",3*s.nodes.size());const auto& v=WallNumbers(n,"velocity_xyz_m_per_s",3*s.nodes.size());
    const auto& q=WallNumbers(n,"orientation_wxyz",4*s.nodes.size());const auto& w=WallNumbers(n,"angular_velocity_xyz_rad_per_s",3*s.nodes.size());
    for(std::size_t i=0;i<s.nodes.size();++i) {
        const auto source=s.nodes[i].position_m;const double initial[]={source.x,source.y,source.z};long double norm=0;
        double displacement=0;
        for(unsigned j=0;j<3;++j) {
            AssemblyEqual(x[3*i+j].GetDouble(),mesh.GetCoordsVertices()[i][j]);
            displacement=std::hypot(displacement,x[3*i+j].GetDouble()-initial[j]);
            if(!e.epoch) {AssemblyEqual(x[3*i+j].GetDouble(),initial[j]);AssemblyEqual(v[3*i+j].GetDouble(),b.source_initial_velocity[j]);
                Require(w[3*i+j].GetDouble()==0,"Initial assembly spin is not zero");}
        }
        Require(displacement<=a.maximum_displacement,"Assembly nodal displacement exceeded declared envelope");
        for(unsigned j=0;j<4;++j) {
            const double value=q[4*i+j].GetDouble();norm+=static_cast<long double>(value)*value;
            if(!e.epoch)Require(value==(j==0?1:0),"Initial assembly orientation changed");
        }
        Require(std::abs(norm-1)<=1e-10L,"Assembly accepted quaternion is not unit length");
    }
    const auto display=CheckAssemblySections(b,e,Member(d,"sections"));const auto& diagnostics=Member(d,"diagnostics");
    double maximum=0;for(const auto& p:display)maximum=std::max(maximum,p.value);
    AssemblyEqual(maximum,Real(diagnostics,"maximum_plastic_strain"));
    const auto& rows=Member(Member(d,"sections"),"sections");long double work=0;std::uint64_t yielded_points=0,yielded_parents=0;
    for(const auto& row:rows.GetArray()) {
        work+=row[0].GetDouble();bool yielded=false;
        for(const auto& p:row[9].GetArray())if(p[5].GetDouble()>0){++yielded_points;yielded=true;}
        yielded_parents+=yielded;
    }
    AssemblyNear(Real(diagnostics,"cumulative_plastic_work_J"),work,s.parents.size());
    Require(Unsigned(diagnostics,"yielded_points")==yielded_points&&Unsigned(diagnostics,"yielded_parents")==yielded_parents,
        "Assembly plastic point/parent summary changed");
    CheckAssemblyDiagnostics(b,e,diagnostics,&n);
    if(e.epoch)CheckAssemblyContact(b,e,Member(d,"contact"),n,diagnostics);
    else Require(Member(d,"contact").IsNull(),"Initial assembly frame cannot publish an interval contact candidate");
    if(e.epoch==b.info.final_epoch)Require(diagnostics==a.final_diagnostics,"Assembly final frame diagnostics differ from final metrics");
}
} // namespace crash::output::replay_detail
