#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
namespace {
void ProjectedFace(const Bundle& b,std::uint64_t face,const Value& point) {
    const auto found=std::find(b.wall_faces.begin(),b.wall_faces.end(),face);
    Require(found!=b.wall_faces.end(),"Assembly contact names a nonexistent original wall triangle");
    const auto& mesh=*b.assembly->wall;const auto& ids=mesh.GetIndicesVertices()[std::size_t(found-b.wall_faces.begin())];
    long double side[3]{};long double scale=0;
    for(unsigned j=0;j<3;++j) {
        const auto& a=mesh.GetCoordsVertices()[ids[j]];const auto& c=mesh.GetCoordsVertices()[ids[(j+1)%3]];
        const long double dy=static_cast<long double>(c.y())-a.y(),dz=static_cast<long double>(c.z())-a.z();
        const long double py=static_cast<long double>(point[1].GetDouble())-a.y(),pz=static_cast<long double>(point[2].GetDouble())-a.z();
        side[j]=dy*pz-dz*py;scale=std::max(scale,std::abs(dy*pz)+std::abs(dz*py));
    }
    const long double tolerance=128.L*std::numeric_limits<double>::epsilon()*scale;
    const bool positive=side[0]>=-tolerance&&side[1]>=-tolerance&&side[2]>=-tolerance;
    const bool negative=side[0]<=tolerance&&side[1]<=tolerance&&side[2]<=tolerance;
    Require(positive||negative,"Assembly projected contact point is outside its actual finite wall triangle");
}
}
void CheckAssemblyContact(const Bundle& b,const Entry& e,const Value& c,const Value& nodal,const Value& diagnostics) {
    const auto& a=*b.assembly;const auto& s=a.source.data();
    Require(Text(c,"phase")=="prepared_candidate_of_accepted_interval"&&Text(c,"certificate_columns")=="value,lower,upper,error"&&
        Unsigned(c,"owner_id")==e.owner&&Unsigned(c,"configuration_id")==b.source_configuration_id&&
        Unsigned(c,"qualification_id")==b.qualification_id&&Unsigned(c,"wall_binding_id")==b.wall_binding_id&&
        Unsigned(c,"base_epoch")==e.epoch-1&&Unsigned(c,"attempt")==e.interval_attempt,
        "Assembly contact is not the candidate of this accepted interval");
    AssemblyEqual(Real(c,"time_s"),e.time);AssemblyEqual(Real(c,"base_time_s"),e.interval_base_time);
    AssemblyEqual(Real(c,"velocity_time_s"),e.interval_base_time+.5*b.fixed_dt);
    AssemblyEqual(Real(c,"base_velocity_time_s"),e.interval_base_velocity_time);
    AssemblyEqual(Real(c,"kick_dt_s"),e.epoch==1?.5*b.fixed_dt:b.fixed_dt);
    const auto resultant=WallCertificate(Member(c,"resultant_N")),potential=WallCertificate(Member(c,"potential_J"));
    for(const char* key:{"wall_reaction_xyz_N","wall_moment_xyz_N_m","wall_kick_moment_xyz_N_m_s","wall_kick_moment_error_xyz_N_m_s"})WallNumbers(c,key,3);
    for(const auto& x:Member(c,"wall_kick_moment_error_xyz_N_m_s").GetArray())Require(x.GetDouble()>=0,"Negative contact moment error");
    for(const char* key:{"maximum_penetration_m","native_mass_stiffness_rate_bound_per_s2","base_potential_J","base_potential_error_J",
        "kick_work_roundoff_J","drift_work_roundoff_J","work_uncertainty_J","quadratic_work_upper_J","wall_kick_impulse_N_s","wall_kick_impulse_error_N_s"})
        Require(Real(c,key)>=0,"Invalid contact magnitude/error");
    for(const char* key:{"surface_power_W","potential_increment_J","kick_work_J","drift_work_J","conservative_defect_J"})Real(c,key);
    AssemblyEqual(Real(c,"base_potential_J"),e.interval_base_wall_potential);
    AssemblyEqual(Real(c,"potential_increment_J"),potential[0]-Real(c,"base_potential_J"));
    AssemblyEqual(Real(c,"conservative_defect_J"),Real(c,"potential_increment_J")+Real(c,"drift_work_J"));
    Require(Real(c,"maximum_penetration_m")<=b.wall_penetration_cap&&
        Real(c,"conservative_defect_J")>=-Real(c,"work_uncertainty_J")&&
        static_cast<long double>(Real(c,"conservative_defect_J"))<=
            static_cast<long double>(Real(c,"quadratic_work_upper_J"))+Real(c,"work_uncertainty_J")+
            std::numeric_limits<double>::epsilon()*std::max(Real(c,"quadratic_work_upper_J"),Real(c,"work_uncertainty_J")),
        "Assembly local contact certificate exceeded its bound");
    Require(Text(c,"node_columns")=="global_node,source_node_id,wall_triangle_id,force_N,potential_J,stiffness_N_m,force_world_xyz_N,wall_point_xyz_m,wall_reaction_xyz_N,wall_moment_xyz_N_m,surface_power_W,fixed,touching_or_penetrating"&&
        Text(c,"parent_columns")=="weight_index,source_parent_index,source_element_id,source_part_id,source_material_id,source_section_id,feature_id,parent_face_id,family,arity,force_N,resultant_N,potential_J",
        "Assembly contact field columns changed");
    const auto& nodes=WallArray(c,"nodes",s.nodes.size());const auto& x=Member(nodal,"position_xyz_m");
    const auto& velocity=Member(nodal,"velocity_xyz_m_per_s");
    std::uint64_t active=0;long double force_sum=0,potential_sum=0,power_sum=0,power_scale=0;
    long double moment_sum[3]{},moment_scale[3]{};
    for(std::size_t n=0;n<s.nodes.size();++n) {
        const auto& row=AssemblyRow(nodes[n],13);
        Require(AssemblyId(row[0])==n&&AssemblyId(row[1])==s.nodes[n].source_id,"Assembly contact unique node mapping changed");
        const auto force=WallCertificate(row[3]),energy=WallCertificate(row[4]);WallCertificate(row[5]);
        const auto& world=AssemblyNumbers(row[6],3);const auto& point=AssemblyNumbers(row[7],3);
        const auto& reaction=AssemblyNumbers(row[8],3);const auto& moment=AssemblyNumbers(row[9],3);const double power=AssemblyNumber(row[10]);
        Require(row[11].IsBool()&&!row[11].GetBool()&&row[12].IsBool(),"Assembly native node fixed/contact flags changed");
        Require(world[0].GetDouble()==-force[0]&&world[1].GetDouble()==0&&world[2].GetDouble()==0&&
            reaction[0].GetDouble()==force[0]&&reaction[1].GetDouble()==0&&reaction[2].GetDouble()==0,
            "Assembly wall force direction changed");
        AssemblyEqual(point[0].GetDouble(),b.wall_x);AssemblyEqual(point[1].GetDouble(),x[3*n+1].GetDouble());
        AssemblyEqual(point[2].GetDouble(),x[3*n+2].GetDouble());ProjectedFace(b,AssemblyId(row[2]),point);
        Require(row[12].GetBool()==(x[3*n].GetDouble()>=b.wall_x),"Assembly touching flag differs from its accepted position");
        const double expected_moment[]={0,point[2].GetDouble()*force[0],-point[1].GetDouble()*force[0]};
        for(unsigned j=0;j<3;++j) {
            Require(moment[j].GetDouble()==expected_moment[j],"Assembly contact moment differs from its actual point and force");
            moment_sum[j]+=moment[j].GetDouble();moment_scale[j]+=std::abs(moment[j].GetDouble());
        }
        const double expected_power=world[0].GetDouble()*velocity[3*n].GetDouble();
        Require(power==expected_power,"Assembly contact surface power differs from candidate force and velocity");
        power_sum+=power;power_scale+=std::abs(power);
        if(force[0]>0){++active;Require(row[12].GetBool(),"Positive wall force on a strictly separated point");}
        force_sum+=force[0];potential_sum+=energy[0];
    }
    Require(active==Unsigned(diagnostics,"active_contact_nodes"),"Assembly contact active unique-node count changed");
    AssemblyNear(resultant[0],force_sum,s.nodes.size());AssemblyNear(potential[0],potential_sum,s.nodes.size());
    const auto& reaction=Member(c,"wall_reaction_xyz_N");
    AssemblyNear(reaction[0].GetDouble(),force_sum,s.nodes.size());Require(reaction[1].GetDouble()==0&&reaction[2].GetDouble()==0,"Assembly wall reaction is not normal");
    for(unsigned j=0;j<3;++j)AssemblyReduction(Member(c,"wall_moment_xyz_N_m")[j].GetDouble(),moment_sum[j],moment_scale[j],s.nodes.size());
    AssemblyReduction(Real(c,"surface_power_W"),power_sum,power_scale,s.nodes.size());
    const auto& parents=WallArray(c,"parents",s.parents.size());
    for(std::size_t i=0;i<s.parents.size();++i) {
        const auto& row=AssemblyRow(parents[i],13);const auto& p=s.parents[a.contact_source_parent[i]];
        const std::uint64_t ids[]={i,p.index,p.source_id,p.part_id,p.material_id,p.section_id,p.source_id,0};
        for(unsigned j=0;j<8;++j)Require(AssemblyId(row[j])==ids[j],"Assembly contact complete parent/family mapping changed");
        Require(row[8].IsString()&&std::string(row[8].GetString())==(p.arity==4?"Q4_center_area":"T3_native")&&AssemblyId(row[9])==p.arity,
            "Assembly contact parent family changed");
        const auto& forces=AssemblyRow(row[10],4);for(unsigned j=0;j<4;++j) {
            const auto f=WallCertificate(forces[j]);if(j>=p.arity)Require(f==std::array<double,4>{},"Unused T3 force slot is not zero");
        }
        WallCertificate(row[11]);WallCertificate(row[12]);
    }
    const auto it=std::find_if(b.entries.begin(),b.entries.end(),[&](const auto& x){return x.epoch==e.epoch;});
    Require(it!=b.entries.end(),"Unindexed assembly contact epoch");const auto& ledger=a.intervals[std::size_t(it-b.entries.begin())];
    const double values[]={resultant[0],resultant[3],potential[0],potential[3],Real(c,"kick_work_J"),Real(c,"drift_work_J"),
        Real(c,"work_uncertainty_J"),Real(c,"quadratic_work_upper_J"),Real(c,"conservative_defect_J"),Real(c,"wall_kick_impulse_N_s"),
        Real(c,"wall_kick_impulse_error_N_s"),Real(c,"maximum_penetration_m")};
    for(unsigned j=0;j<12;++j)AssemblyEqual(values[j],ledger[j+14]);
}
} // namespace crash::output::replay_detail
