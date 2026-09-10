#include "AcceptedReplaySourcePartWall.h"
#include "SourcePartWallArtifactSchema.h"
#include <algorithm>
#include <set>
namespace crash::output::replay_detail {
void ReadSourcePartWallConfiguration(Bundle& b,const Document& c,const Document& final,const Document& manifest) {
    Require(Text(c,"schema")=="robo_dyna.source_part_wall_configuration.v1"&&Unsigned(c,"owner_id")==b.info.owner_id&&
        Unsigned(final,"owner_id")==b.info.owner_id&&Unsigned(final,"saved_frames")==b.info.frame_count,"Wall configuration association failed");
    Require(Text(c,"source_readiness_sha256")=="74e733b76a5c530c94ba39876ce3707df2ca8c602348204632eb902a48402d89"&&
        Unsigned(c,"source_readiness_bytes")==671971&&Unsigned(c,"source_part_id")==2000157&&Unsigned(c,"q4_count")==88&&Unsigned(c,"t3_count")==6&&
        !b.inventory.count("canonical-wall.mesh.json"),"Wall replay original source provenance/scope mismatch");
    WallBool(manifest,"contact",true);const auto& complete=Member(manifest,"horizon_complete");Require(complete.IsBool(),"Wall horizon flag is invalid");
    b.info.horizon_complete=complete.GetBool();b.info.stop_reason=Text(manifest,"stop_reason");const auto requested=Unsigned(c,"required_steps");
    Require(requested&&Unsigned(manifest,"requested_steps")==requested&&b.info.final_epoch<=requested&&
        (b.info.horizon_complete?b.info.final_epoch==requested:b.info.final_epoch<requested)&&
        (b.info.horizon_complete?b.info.stop_reason.empty():!b.info.stop_reason.empty()),"Wall prefix/horizon completion claim is inconsistent");
    CheckReplayTime(Real(c,"requested_horizon_s"),requested*b.fixed_dt,b.fixed_dt,requested);
    Require(Bits(Real(c,"young_modulus_Pa"))==Bits(200e9)&&Bits(Real(c,"poisson_ratio"))==Bits(.3)&&
        Real(c,"density_kg_m3")>0&&Real(c,"thickness_m")>0&&!Text(c,"attachment_policy").empty()&&!Text(c,"mass_policy").empty(),
        "Wall experimental source material/mass declaration is missing");
    b.info.run_id=Unsigned(c,"run_id");b.info.topology_id=Unsigned(c,"topology_id");b.source_configuration_id=Unsigned(c,"configuration_id");
    b.qualification_id=Unsigned(c,"qualification_id");Require(b.info.run_id&&b.info.topology_id&&b.source_configuration_id&&b.qualification_id,"Zero wall run identity");
    ReadSourcePartIdentity(b,c);const auto& setup=Member(c,"wall_setup");Require(setup.IsObject(),"Missing wall setup certificate");
    Require(Text(setup,"contact_model")=="reference-area-lumped-nodal-wall-v1"&&Unsigned(setup,"configuration_id")==b.source_configuration_id&&
        Unsigned(setup,"qualification_id")==b.qualification_id&&Bits(Real(setup,"fixed_dt_s"))==Bits(b.fixed_dt),"Wall setup model/identity mismatch");
    b.wall_binding_id=Unsigned(setup,"wall_binding_id");Require(b.wall_binding_id,"Zero wall binding identity");
    const auto& velocity=WallNumbers(c,"initial_velocity_xyz_m_per_s",3);const auto& setup_velocity=WallNumbers(setup,"initial_velocity_xyz_m_per_s",3);
    for(unsigned j=0;j<3;++j){b.source_initial_velocity[j]=velocity[j].GetDouble();Require(Bits(velocity[j].GetDouble())==Bits(setup_velocity[j].GetDouble()),"Wall startup velocity declaration changed");}
    Require(b.source_initial_velocity[0]>0&&b.source_initial_velocity[1]==0&&b.source_initial_velocity[2]==0,"Unsupported wall startup direction");
    b.source_initial_kinetic=Real(c,"initial_kinetic_J");const auto kinetic=WallCertificate(Member(setup,"native_initial_kinetic_J"));
    Require(b.source_initial_kinetic>0&&Bits(b.source_initial_kinetic)==Bits(Real(setup,"measured_initial_kinetic_J"))&&
        Bits(b.source_initial_kinetic)==Bits(Real(final,"initial_kinetic_J"))&&b.source_initial_kinetic>=kinetic[1]&&b.source_initial_kinetic<=kinetic[2],
        "Measured source initial energy is not associated with its native certificate");
    // The same bounded ordered native scalar sum used by common startup.
    // This checks archived mass/velocity association, without shell mechanics.
    double ordered_kinetic=0;const double speed=b.source_initial_velocity[0];
    for(const auto& node:WallArray(c,"reference_nodes",117).GetArray())ordered_kinetic+=.5*Real(node,"mass_kg")*(speed*speed);
    Require(std::isfinite(ordered_kinetic)&&Bits(ordered_kinetic)==Bits(b.source_initial_kinetic)&&
        ordered_kinetic>=kinetic[1]&&ordered_kinetic<=kinetic[2],"Archived native mass reduction differs from measured initial K0");
    const double floor=Real(c,"maximum_energy_residual_J"),relative=Real(c,"relative_energy_residual");
    Require(floor>0&&relative>0,"Wall fixed physical energy budget is invalid");b.wall_energy_allowance=floor+relative*b.source_initial_kinetic;
    Require(std::isfinite(b.wall_energy_allowance)&&b.wall_energy_allowance>0,"Wall physical budget overflow");
    const double area_floor=Real(setup,"area_floor_m2"),design=Real(setup,"design_penetration_m");b.wall_penetration_cap=Real(setup,"penetration_cap_m");
    Require(area_floor>0&&design>0&&design<b.wall_penetration_cap&&Real(setup,"kinetic_budget_factor")>1&&
        Real(setup,"kinetic_budget_upper_J")>=kinetic[2]&&Real(setup,"stiffness_per_area_N_m3")>0&&
        Real(setup,"design_potential_lower_J")>=Real(setup,"kinetic_budget_upper_J")&&Real(setup,"minimum_nodal_area_lower_m2")>=area_floor,
        "Wall fixed penalty design certificate is invalid");
    for(const char* key:{"declared_leading_gap_m","motion_margin_m","exposed_clearance_m","parent_force_error_N","parent_energy_error_J","maximum_contact_step_rate"})
        Require(Real(setup,key)>0,"Wall physical setting must be positive");
    Require(Real(setup,"maximum_contact_step_rate")<=.125,"Wall contact-only timestep guard is invalid");WallBool(setup,"projected_motion_covered",true);
    const auto& areas=WallArray(setup,"nodal_area_m2",117);double minimum=HUGE_VAL;unsigned minimum_node=0;
    for(unsigned n=0;n<117;++n){const auto area=WallCertificate(areas[n]);Require(area[1]>=area_floor,"A source node lacks its certified contact area floor");
        if(area[1]<minimum){minimum=area[1];minimum_node=n;}}
    Require(Bits(minimum)==Bits(Real(setup,"minimum_nodal_area_lower_m2"))&&minimum_node==Unsigned(setup,"minimum_area_node"),"Minimum source area identity changed");
    const auto& parents=WallArray(setup,"contact_parents",94);const auto& source=WallArray(c,"source_parents",94);std::set<std::uint64_t> mapped;
    for(unsigned p=0;p<94;++p){const auto index=Unsigned(parents[p],"source_parent_index"),eid=Unsigned(parents[p],"source_element_id"),arity=Unsigned(parents[p],"arity");
        Require(index<94&&mapped.insert(index).second&&Unsigned(parents[p],"weight_index")==p&&eid==Unsigned(source[index],"source_element_id")&&
            arity==Unsigned(source[index],"arity"),"Contact sorted-parent/source mapping changed");
        WallCertificate(Member(parents[p],"area_m2"));WallCertificate(Member(parents[p],"share_m2"));b.wall_source_parents.push_back({eid,arity,index});}
    CheckPlacedSourcePartWall(b,c);ReadSourcePartWallIntervals(b,c,manifest);
}
void CheckSourcePartWallFields(const Bundle& b,const Entry& e,const chrono::ChTriangleMeshConnected& mesh) {
    const auto name=e.mesh.substr(0,e.mesh.size()-10)+".fields.json";
    const auto fields=b.inventory.find(name),mesh_file=b.inventory.find(e.mesh),obj=b.inventory.find(e.obj);
    Require(fields!=b.inventory.end()&&fields->second.bytes<=SourcePartWallFieldCap&&
        mesh_file!=b.inventory.end()&&mesh_file->second.bytes<=SourcePartWallMeshCap&&
        obj!=b.inventory.end()&&obj->second.bytes<=SourcePartWallObjCap,"Wall frame exceeds its schema byte cap");
    const auto f=Json(VerifiedBytes(b,name));
    Require(Text(f,"schema")=="robo_dyna.source_part_wall_fields.v1","Wall field schema mismatch");
    CheckSourcePartFieldData(b,e,mesh,f,b.source_initial_velocity);
    Require(Real(f,"external_kick_work_J")==0&&Real(f,"external_drift_work_J")==0&&Real(f,"absolute_external_drift_work_J")==0,
        "Wall impact fields contain an undeclared external pulse");
    for(const char* key:{"carried_momentum_residual_xyz_kg_m_per_s","carried_momentum_allowance_xyz_kg_m_per_s",
        "carried_angular_momentum_xyz_kg_m2_per_s","cumulative_wall_kick_moment_xyz_N_m_s","cumulative_wall_kick_moment_error_xyz_N_m_s"})WallNumbers(f,key,3);
    Require(!Text(f,"angular_momentum_timing").empty(),"Wall angular reporting timing is missing");
    for(const char* key:{"cumulative_wall_kick_impulse_N_s","cumulative_wall_kick_impulse_error_N_s","synchronized_kinetic_uncertainty_J",
        "physical_energy_uncertainty_J","energy_allowance_J"})Require(Real(f,key)>=0,"Wall uncertainty or impulse is negative");
    const auto first=Unsigned(f,"first_contact_epoch"),last=Unsigned(f,"last_contact_epoch"),count=Unsigned(f,"contact_intervals"),active=Unsigned(f,"active_nodes");
    Require(active<=117&&count<=e.epoch&&last<=e.epoch&&((count==0&&first==0&&last==0)||(count>0&&first>0&&first<=last)),"Wall contact event metadata is inconsistent");
    if(!e.epoch) {
        Require(Text(f,"contact_state")=="certified_separated_startup"&&Member(f,"contact").IsNull()&&count==0&&active==0&&
            Real(f,"cumulative_wall_kick_impulse_N_s")==0&&Real(f,"cumulative_wall_kick_impulse_error_N_s")==0&&Unsigned(f,"strictly_separated_nodes")==117,
            "Wall startup fabricated an interval contact result");
        Require(Bits(Real(Member(f,"carried_kinetic"),"translation_J"))==Bits(b.source_initial_kinetic)&&
            Real(Member(f,"carried_kinetic"),"rotation_total_J")==0,"Wall startup common kinetic energy changed");return;
    }
    Require(Text(f,"contact_state")=="committed_interval_candidate","Wall accepted contact provenance was relabeled");
    const auto& c=Member(f,"contact");const auto separated=CheckSourcePartWallContact(b,e,c);
    Require(separated==Unsigned(f,"strictly_separated_nodes"),"Strict source separation count disagrees with the actual copied nodes");
    const auto force=WallCertificate(Member(c,"resultant_N")),potential=WallCertificate(Member(c,"potential_J"));
    const std::array<double,28> values{Real(f,"velocity_time_s"),Real(f,"reaction_kick_dt_s"),Real(f,"synchronized_kinetic_J"),Real(f,"total_internal_work_J"),
        Real(f,"energy_residual_J"),Real(f,"kinetic_work_residual_J"),Real(f,"kinetic_work_allowance_J"),force[0],force[3],potential[0],potential[3],
        Real(c,"kick_work_J"),Real(c,"drift_work_J"),Real(c,"work_uncertainty_J"),Real(c,"quadratic_work_upper_J"),Real(c,"wall_kick_impulse_N_s"),
        Real(c,"wall_kick_impulse_error_N_s"),Real(f,"cumulative_wall_kick_impulse_N_s"),Real(f,"cumulative_wall_kick_impulse_error_N_s"),
        Real(c,"maximum_penetration_m"),double(active),Real(f,"synchronized_kinetic_uncertainty_J"),Real(f,"energy_allowance_J"),
        f["carried_momentum_residual_xyz_kg_m_per_s"][0].GetDouble(),f["carried_momentum_allowance_xyz_kg_m_per_s"][0].GetDouble(),
        Real(f,"maximum_rotation_rad"),Real(f,"physical_energy_uncertainty_J"),double(Unsigned(f,"strictly_separated_nodes"))};
    for(unsigned i=0;i<values.size();++i)Require(Bits(values[i])==Bits(e.wall_interval_values[i]),"Wall saved scientific fields differ from their accepted interval ledger");
    Require(Bits(Real(f,"energy_allowance_J"))==Bits(b.wall_energy_allowance)&&
        std::abs(Real(f,"energy_residual_J"))+Real(f,"physical_energy_uncertainty_J")<=b.wall_energy_allowance,
        "Wall physical energy uncertainty was treated as extra allowance");
}
}
