#include "AcceptedReplaySourcePartWall.h"
#include <algorithm>
namespace crash::output::replay_detail {
unsigned CheckSourcePartWallContact(const Bundle& b,const Entry& e,const Value& c) {
    Require(c.IsObject()&&Text(c,"phase")=="prepared_candidate_of_committed_interval"&&Unsigned(c,"owner_id")==e.owner&&
        Unsigned(c,"configuration_id")==b.source_configuration_id&&Unsigned(c,"qualification_id")==b.qualification_id&&
        Unsigned(c,"wall_binding_id")==b.wall_binding_id&&Unsigned(c,"base_epoch")==e.epoch-1&&Unsigned(c,"attempt")==e.interval_attempt&&
        Text(c,"temporal_scheme")=="staggered_half_kick_start"&&Text(c,"velocity_phase")=="previous_midpoint"&&
        Bits(Real(c,"time_s"))==Bits(e.time)&&Bits(Real(c,"base_time_s"))==Bits(e.interval_base_time)&&
        Bits(Real(c,"velocity_time_s"))==Bits(e.interval_base_time+.5*b.fixed_dt)&&
        Bits(Real(c,"base_velocity_time_s"))==Bits(e.epoch==1?0:e.interval_base_time-.5*b.fixed_dt)&&
        Bits(Real(c,"kick_dt_s"))==Bits(e.epoch==1?.5*b.fixed_dt:b.fixed_dt),"Wall contact copy is not this accepted interval candidate");
    Require(Unsigned(c,"node_count")==117&&Unsigned(c,"parent_count")==94,"Wall contact source union changed");
    WallCertificate(Member(c,"resultant_N"));WallCertificate(Member(c,"potential_J"));
    for(const char* key:{"wall_reaction_xyz_N","wall_moment_xyz_N_m","wall_kick_moment_xyz_N_m_s","wall_kick_moment_error_xyz_N_m_s"})WallNumbers(c,key,3);
    for(const char* key:{"surface_power_W","stiffness_rate_bound_s2","base_potential_J","potential_increment_J","kick_work_J","drift_work_J","conservative_defect_J"})Real(c,key);
    for(const char* key:{"base_potential_error_J","kick_work_roundoff_J","drift_work_roundoff_J","work_uncertainty_J","quadratic_work_upper_J",
        "wall_kick_impulse_N_s","wall_kick_impulse_error_N_s"})Require(Real(c,key)>=0,"Wall contact error or impulse is negative");
    Require(Real(c,"maximum_penetration_m")>=0&&Real(c,"maximum_penetration_m")<=b.wall_penetration_cap,"Wall contact penetration exceeded the archived cap");
    const auto& nodes=WallArray(c,"nodes",117);const auto& parents=WallArray(c,"parents",94);
    unsigned strictly_separated=0;
    for(unsigned n=0;n<117;++n) {
        const auto& a=nodes[n];Require(Unsigned(a,"node")==n,"Wall contact node order changed");
        const auto face=Unsigned(a,"wall_face");Require(std::find(b.wall_faces.begin(),b.wall_faces.end(),face)!=b.wall_faces.end(),"Wall contact node names a nonexistent actual mesh face");
        WallBool(a,"fixed",false);Require(Member(a,"touching_or_penetrating").IsBool(),"Wall contact activity flag is invalid");
        const auto force=WallCertificate(Member(a,"force_N")),potential=WallCertificate(Member(a,"potential_J"));WallCertificate(Member(a,"stiffness_N_m"));
        strictly_separated+=!Member(a,"touching_or_penetrating").GetBool()&&force[2]==0&&potential[2]==0;
        const auto& point=WallNumbers(a,"wall_point_xyz_m",3);const auto& world=WallNumbers(a,"force_world_xyz_N",3);
        const auto& reaction=WallNumbers(a,"wall_reaction_xyz_N",3);WallNumbers(a,"wall_moment_xyz_N_m",3);Real(a,"surface_power_W");
        Require(Bits(point[0].GetDouble())==Bits(b.wall_x)&&world[0].GetDouble()==-force[0]&&world[1].GetDouble()==0&&world[2].GetDouble()==0&&
            reaction[0].GetDouble()==force[0]&&reaction[1].GetDouble()==0&&reaction[2].GetDouble()==0,"Wall node force direction or placed point changed");
    }
    for(unsigned p=0;p<94;++p) {
        const auto& a=parents[p];const auto expected=b.wall_source_parents[p];
        Require(Unsigned(a,"source_element_id")==expected[0]&&Unsigned(a,"feature_id")==expected[0]&&Unsigned(a,"parent_face_id")==0&&
            Unsigned(a,"arity")==expected[1]&&Text(a,"family")== (expected[1]==4?"Q4_center_area":"T3_native"),"Wall contact parent identity/family changed");
        const auto& force=WallArray(a,"force_N",4);for(unsigned j=0;j<4;++j){const auto f=WallCertificate(force[j]);
            if(j>=expected[1])Require(f==std::array<double,4>{},"Unused T3 fourth contact slot is not zero");}
        WallCertificate(Member(a,"resultant_N"));WallCertificate(Member(a,"potential_J"));
    }
    // Structural associations and recorded certificate shapes are checked.
    // Replay does not re-run shell/contact mechanics or admit a new timestep.
    return strictly_separated;
}
}
