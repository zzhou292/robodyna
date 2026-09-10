#include "AcceptedReplayData.h"
#include "CsvLedgerSegments.h"
#include "SourcePartArtifactSchema.h"
#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace crash::output::replay_detail {
namespace {
constexpr std::size_t Nodes=117,Parents=94,Q4=88,T3=6;
constexpr std::uint64_t Part=2000157;
constexpr const char* Readiness="74e733b76a5c530c94ba39876ce3707df2ca8c602348204632eb902a48402d89";
void Bool(const Value& object,const char* key,bool expected) {
    const auto& v=Member(object,key);Require(v.IsBool()&&v.GetBool()==expected,"Source replay boolean association failed");
}
const Value& Array(const Value& object,const char* key,std::size_t count) {
    const auto& a=Member(object,key);Require(a.IsArray()&&a.Size()==count,"Source replay array shape failed");return a;
}
void Finite(const Value& object,const char* key,std::size_t count) {
    for(const auto& v:Array(object,key,count).GetArray())
        Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Source replay field contains a nonfinite value");
}
template<class T> T Csv(const std::string& text) {
    T value{}; const auto p=std::from_chars(text.data(),text.data()+text.size(),value);
    Require(p.ec==std::errc{}&&p.ptr==text.data()+text.size(),"Malformed source interval number");return value;
}
void ReadIntervals(Bundle& bundle,const Document& config,const Document& manifest) {
    const auto plans=ParseCsvLedgerSegments(Member(config,kCsvLedgerSegmentsField));
    const auto completed=ParseCsvLedgerSegments(Member(manifest,kCsvLedgerSegmentsField));
    Require(plans.size()==1&&completed.size()==1&&SameCsvLedgerPlan(plans[0],completed[0]),"Source interval segment association failed");
    const auto& plan=plans[0]; ValidateCsvLedgerPlan(plan,SourcePartIntervalHeader);
    Require(plan.logical_file=="accepted-intervals.csv"&&plan.interval_count==bundle.info.final_epoch&&plan.column_count==27,
        "Source interval plan has an incorrect horizon or shape");
    std::uint64_t previous=0; double previous_time=0; std::size_t frame=1;
    for(const auto& segment:plan.segments) {
        std::istringstream input(VerifiedBytes(bundle,segment.file)); std::string line;
        Require(bool(std::getline(input,line))&&line+"\n"==SourcePartIntervalHeader,"Source interval header mismatch");
        std::uint64_t count=0;
        while(std::getline(input,line)) {
            std::array<std::string,27> c;std::istringstream row(line);
            for(auto& v:c)Require(bool(std::getline(row,v,','))&&!v.empty(),"Incomplete source interval");
            Require(row.eof(),"Extra source interval column");
            const auto owner=Csv<std::uint64_t>(c[0]),base=Csv<std::uint64_t>(c[1]),attempt=Csv<std::uint64_t>(c[2]),epoch=Csv<std::uint64_t>(c[4]);
            const double base_time=Csv<double>(c[3]),time=Csv<double>(c[5]),velocity_time=Csv<double>(c[6]),kick=Csv<double>(c[7]);
            Require(owner==bundle.info.owner_id&&base==previous&&epoch==base+1&&attempt&&Bits(base_time)==Bits(previous_time)&&
                Bits(time)==Bits(base_time+bundle.fixed_dt)&&Bits(velocity_time)==Bits(base_time+.5*bundle.fixed_dt)&&
                Bits(kick)==Bits(base==0?.5*bundle.fixed_dt:bundle.fixed_dt),"Source interval identity or phase mismatch");
            for(std::size_t i=8;i<c.size();++i)Require(std::isfinite(Csv<double>(c[i])),"Nonfinite source interval diagnostic");
            if(frame<bundle.entries.size()&&bundle.entries[frame].epoch==epoch) {
                Require(Bits(bundle.entries[frame].time)==Bits(time),"Source saved frame interval time mismatch");
                bundle.entries[frame].interval_attempt=attempt;bundle.entries[frame].interval_base_time=base_time;++frame;
            }
            previous=epoch;previous_time=time;++count;
            Require(count<=segment.row_count&&epoch<=segment.last_epoch,"Source interval segment overflow");
        }
        Require(count==segment.row_count&&previous==segment.last_epoch,"Source interval segment incomplete");
    }
    Require(previous==bundle.info.final_epoch&&Bits(previous_time)==Bits(bundle.info.final_time)&&frame==bundle.entries.size(),
        "Source intervals do not reach every frame and the completed horizon");
}
void Family(const Bundle& bundle,const Entry& entry,const Value& d) {
    Bool(d,"valid",true);Bool(d,"kinetic_available",false);Bool(d,"has_completed_interval",entry.epoch!=0);
    Require(Unsigned(d,"owner_id")==entry.owner&&Unsigned(d,"configuration_id")==bundle.source_configuration_id&&
        Unsigned(d,"qualification_id")==bundle.qualification_id&&Unsigned(d,"epoch")==entry.epoch&&Bits(Real(d,"time_s"))==Bits(entry.time),
        "Source family endpoint association mismatch");
    if(entry.epoch) {
        Bool(d,"accepted_force_assembled",true);
        Require(Unsigned(d,"base_epoch")==entry.epoch-1&&Unsigned(d,"attempt")==entry.interval_attempt&&
            Bits(Real(d,"base_time_s"))==Bits(entry.interval_base_time)&&
            Bits(Real(d,"velocity_time_s"))==Bits(entry.interval_base_time+.5*bundle.fixed_dt)&&
            Bits(Real(d,"kick_dt_s"))==Bits(entry.epoch==1?.5*bundle.fixed_dt:bundle.fixed_dt),"Source family interval association mismatch");
    }
    Finite(d,"internal_work_J",2);Finite(d,"internal_work_increment_J",2);
    for(const char* name:{"internal_kick_work_J","internal_drift_work_J","minimum_area_ratio","minimum_thickness_ratio",
        "maximum_displacement_m","maximum_absolute_strain","maximum_thickness_curvature","minimum_native_dt_s"})Real(d,name);
}
}
void ReadSourcePartIdentity(Bundle& bundle,const Document& c) {
    const auto& vertices=Array(c,"vertex_binding",Nodes);const auto& nodes=Array(c,"reference_nodes",Nodes);
    const auto& parents=Array(c,"source_parents",Parents);const auto& triangles=Array(c,"triangle_binding",2*Q4+T3);
    bundle.info.node_count=Nodes;std::set<std::uint64_t> node_ids,parent_ids;
    for(std::size_t n=0;n<Nodes;++n) {
        const auto& v=vertices[n];Require(v.IsArray()&&v.Size()==4,"Source vertex binding shape mismatch");
        for(const auto& k:v.GetArray())Require(k.IsUint64(),"Source vertex identifier type mismatch");
        Require(v[0].GetUint64()==n&&v[1].GetUint64()==1&&v[2].GetUint64()==1&&v[3].GetUint64()&&node_ids.insert(v[3].GetUint64()).second&&
            Unsigned(nodes[n],"local_node")==n&&Unsigned(nodes[n],"source_node_id")==v[3].GetUint64(),"Source node identity mismatch");
        Finite(nodes[n],"reference_xyz_m",3);
        Require(Real(nodes[n],"mass_kg")>0&&Real(nodes[n],"isotropic_inertia_kg_m2")>0&&Real(nodes[n],"physical_inertia_kg_m2")>0&&
            Real(nodes[n],"added_inertia_kg_m2")>=0,"Source native structural mass/inertia is invalid");
    }
    std::size_t qi=0,ti=0,display=0;std::array<bool,Nodes> covered{};
    for(std::size_t p=0;p<Parents;++p) {
        const auto& item=parents[p];const auto eid=Unsigned(item,"source_element_id"),arity=Unsigned(item,"arity");
        Require(Unsigned(item,"source_parent_index")==p&&eid&&parent_ids.insert(eid).second&&(arity==3||arity==4),"Source parent identity mismatch");
        Require(Text(item,"family")== (arity==4?"QEPH":"T3")&&Unsigned(item,"family_index")== (arity==4?qi++:ti++),"Source family mapping mismatch");
        const auto& connectivity=Array(item,"local_connectivity",arity);std::set<unsigned> unique;
        for(const auto& index:connectivity.GetArray()) {Require(index.IsUint()&&index.GetUint()<Nodes&&unique.insert(index.GetUint()).second,"Source connectivity invalid");covered[index.GetUint()]=true;}
        for(unsigned sub=0;sub<(arity==4?2:1);++sub) {
            Require(display<triangles.Size(),"Source display triangle count overflow");
            const auto& row=triangles[display++];Require(row.IsArray()&&row.Size()==9,"Source triangle mapping shape mismatch");
            for(const auto& value:row.GetArray())Require(value.IsUint64(),"Source triangle identifier type mismatch");
            const unsigned local[]{0,sub?2u:1u,sub?3u:2u};std::array<int,3> face{};std::array<std::uint64_t,9> source{};
            for(unsigned j=0;j<3;++j) {Require(row[j].GetUint64()==connectivity[local[j]].GetUint(),"Source display triangle changed original parent order");face[j]=row[j].GetUint();}
            Require(row[3].GetUint64()==1&&row[4].GetUint64()==1&&row[5].GetUint64()==eid&&row[6].GetUint64()==Part&&row[7].GetUint64()==0&&row[8].GetUint64()==sub,
                "Source display triangle parent identity mismatch");
            for(unsigned j=0;j<9;++j)source[j]=row[j].GetUint64();bundle.topology.push_back(face);bundle.source_triangles.push_back(source);
        }
    }
    Require(qi==Q4&&ti==T3&&display==triangles.Size(),"Source family counts changed");
    for(bool present:covered)Require(present,"Source node was omitted from mechanics connectivity");
}
void ReadSourcePartConfiguration(Bundle& bundle,const Document& c,const Document& final,const Document& manifest) {
    Require(Text(c,"schema")=="robo_dyna.source_part_elastic_configuration.v1"&&Unsigned(c,"owner_id")==bundle.info.owner_id&&
        Unsigned(c,"required_steps")==bundle.info.final_epoch&&Unsigned(final,"saved_frames")==bundle.info.frame_count&&
        Unsigned(final,"owner_id")==bundle.info.owner_id,"Source replay configuration association mismatch");
    Require(Text(c,"source_readiness_sha256")==Readiness&&Unsigned(c,"source_readiness_bytes")==671971&&Unsigned(c,"source_part_id")==Part&&
        Unsigned(c,"q4_count")==Q4&&Unsigned(c,"t3_count")==T3&&!bundle.inventory.count("canonical-wall.mesh.json"),"Source replay scope or provenance mismatch");
    Bool(manifest,"contact",false);Require(!Text(c,"attachment_policy").empty()&&!Text(c,"material_policy").empty(),"Source modeling policy is missing");
    Require(Bits(Real(c,"young_modulus_Pa"))==Bits(200e9)&&Bits(Real(c,"poisson_ratio"))==Bits(.3)&&
        Real(c,"density_kg_m3")>0&&Real(c,"thickness_m")>0,"Source experimental material declaration mismatch");
    bundle.info.run_id=Unsigned(c,"run_id");bundle.info.topology_id=Unsigned(c,"topology_id");
    bundle.qualification_id=Unsigned(c,"qualification_id");bundle.source_configuration_id=Unsigned(c,"configuration_id");
    Require(bundle.info.run_id&&bundle.info.topology_id&&bundle.qualification_id&&bundle.source_configuration_id,"Source experiment identity is zero");
    ReadSourcePartIdentity(bundle,c);
    ReadIntervals(bundle,c,manifest);
}
void CheckSourcePartFieldData(const Bundle& bundle,const Entry& entry,const chrono::ChTriangleMeshConnected& mesh,
    const Document& f,const std::array<double,3>& startup_velocity) {
    Require(Unsigned(f,"owner_id")==entry.owner&&
        Unsigned(f,"accepted_epoch")==entry.epoch&&Bits(Real(f,"accepted_time_s"))==Bits(entry.time)&&Bits(Real(f,"fixed_dt_s"))==Bits(bundle.fixed_dt)&&
        Text(f,"temporal_scheme")=="staggered_half_kick_start","Source field owner/epoch/scheme mismatch");
    CheckPositionFields(f,mesh);Finite(f,"orientation_wxyz",4*Nodes);Finite(f,"velocity_xyz_m_per_s",3*Nodes);
    Finite(f,"omega_world_xyz_rad_per_s",3*Nodes);Finite(f,"synchronized_velocity_xyz_m_per_s",3*Nodes);
    Finite(f,"synchronized_omega_world_xyz_rad_per_s",3*Nodes);
    for(std::size_t n=0;n<Nodes;++n) {double length2=0;for(unsigned j=0;j<4;++j){const double x=f["orientation_wxyz"][4*n+j].GetDouble();length2+=x*x;}
        Require(std::abs(length2-1)<=1e-12,"Source orientation is not a unit quaternion");}
    Bool(f,"reactions_valid",entry.epoch!=0);Bool(f,"shell_diagnostics_valid",true);
    if(!entry.epoch) {
        Require(Text(f,"velocity_phase")=="collocated"&&Real(f,"velocity_time_s")==0&&Real(f,"reaction_time_s")==0&&
            Real(f,"reaction_kick_dt_s")==0&&Unsigned(f,"reaction_base_epoch")==0,"Source initial fields are not physical collocated startup");
        const auto c=Json(VerifiedBytes(bundle,"configuration.json"));
        for(std::size_t n=0;n<Nodes;++n) for(unsigned j=0;j<3;++j)
            Require(Bits(mesh.GetCoordsVertices()[n][j])==Bits(c["reference_nodes"][n]["reference_xyz_m"][j].GetDouble())&&
                f["velocity_xyz_m_per_s"][3*n+j].GetDouble()==startup_velocity[j]&&f["omega_world_xyz_rad_per_s"][3*n+j].GetDouble()==0&&
                f["synchronized_velocity_xyz_m_per_s"][3*n+j].GetDouble()==startup_velocity[j]&&f["synchronized_omega_world_xyz_rad_per_s"][3*n+j].GetDouble()==0,
                "Source startup geometry/velocity changed");
        for(std::size_t n=0;n<Nodes;++n)for(unsigned j=0;j<4;++j)
            Require(f["orientation_wxyz"][4*n+j].GetDouble()==(j==0?1:0),"Source initial orientation is not identity");
    } else Require(Text(f,"velocity_phase")=="previous_midpoint"&&Unsigned(f,"reaction_base_epoch")==entry.epoch-1&&
        Bits(Real(f,"reaction_time_s"))==Bits(entry.interval_base_time)&&Bits(Real(f,"velocity_time_s"))==Bits(entry.interval_base_time+.5*bundle.fixed_dt)&&
        Bits(Real(f,"reaction_kick_dt_s"))==Bits(entry.epoch==1?.5*bundle.fixed_dt:bundle.fixed_dt),"Source raw velocity phase or first kick was relabeled");
    Family(bundle,entry,Member(f,"qeph"));Family(bundle,entry,Member(f,"t3"));
    for(const char* key:{"external_kick_work_J","external_drift_work_J","absolute_external_drift_work_J","kinetic_work_residual_J",
        "kinetic_work_allowance_J","synchronized_kinetic_J","total_internal_work_J","energy_residual_J","maximum_relative_displacement_m",
        "maximum_chord_change_m","maximum_rotation_rad","maximum_area_ratio","maximum_thickness_ratio"})Real(f,key);
    for(const char* key:{"base_kinetic","carried_kinetic"})for(const char* component:{"translation_J","rotation_total_J","rotation_physical_isotropic_J","rotation_added_isotropic_J"})
        Require(Real(Member(f,key),component)>=0,"Source kinetic diagnostic is negative");
    Real(Member(f,"qeph"),"hourglass_viscous_work_J");Real(Member(f,"qeph"),"hourglass_viscous_work_increment_J");
}
void CheckSourcePartFields(const Bundle& bundle,const Entry& entry,const chrono::ChTriangleMeshConnected& mesh) {
    const auto f=Json(VerifiedBytes(bundle,entry.mesh.substr(0,entry.mesh.size()-10)+".fields.json"));
    Require(Text(f,"schema")=="robo_dyna.source_part_elastic_fields.v1","Source pulse field schema mismatch");
    CheckSourcePartFieldData(bundle,entry,mesh,f,{});
}
} // namespace crash::output::replay_detail
