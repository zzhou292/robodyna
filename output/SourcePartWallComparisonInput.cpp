#include "SourcePartWallComparisonInput.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <iomanip>
#include <sstream>

namespace crash::output::wall_comparison {
const Value& Field(const Value& v,const char* key) { return source_comparison::Field(v,key); }
double Real(const Value& v,const char* key) {
    const auto& field=Field(v,key);Require(field.IsNumber()&&std::isfinite(field.GetDouble()),"Invalid wall comparison scalar");return field.GetDouble();
}
std::uint64_t Unsigned(const Value& v,const char* key) {
    const auto& field=Field(v,key);Require(field.IsUint64(),"Invalid wall comparison integer");return field.GetUint64();
}
std::string Encode(const Value& v) {
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(v.Accept(writer),"Cannot encode wall comparison input");return {buffer.GetString(),buffer.GetSize()};
}
Document ReadDocument(const std::filesystem::path& path) {
    const auto bytes=ReadBounded(path,32*1024*1024);Document d;
    d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    Require(!d.HasParseError()&&d.IsObject(),"Invalid wall comparison document");return d;
}
Document PhysicalConfiguration(const Value& config) {
    Require(config.IsObject(),"Invalid physical configuration");
    Document d;d.CopyFrom(config,d.GetAllocator());
    // Compare every physical/provenance field, including future extensions.
    // Only the explicit refinement/output identities are excluded.
    for(const char* key:{"fixed_dt_s","required_steps","frame_every","owner_id","run_id","topology_id","interval_ledger_segments"})d.RemoveMember(key);
    Require(d.HasMember("wall_setup")&&d["wall_setup"].IsObject(),"Missing physical wall setup");
    d["wall_setup"].RemoveMember("fixed_dt_s");return d;
}
void ValidatePilot(const Value& config,unsigned refinement) {
    Require(refinement==1||refinement==2||refinement==4,"Unknown wall refinement");
    Require(Bits(Real(config,"fixed_dt_s"))==Bits(BaseStep/refinement)&&
        Bits(Real(config,"requested_horizon_s"))==Bits(BaseStep*BaseHorizon)&&
        Unsigned(config,"required_steps")==BaseHorizon*refinement,"Comparison requires the frozen full horizon");
    Require(Unsigned(config,"frame_every")==CommonStride*refinement,"Wall comparison changed the common sample cadence");
    const auto exact=[&](const Value& object,const char* key,double expected) {
        Require(Bits(Real(object,key))==Bits(expected),"Wall run changed the frozen physical tuple");
    };
    for(const auto& item:std::array<std::pair<const char*,double>,16>{{
        {"young_modulus_Pa",200e9},{"poisson_ratio",.3},{"density_kg_m3",7.889999999999999090505e3},
        {"thickness_m",1.647999999999999942366e-3},{"maximum_displacement_m",.02},{"maximum_rotation_rad",.25},
        {"maximum_strain",.005},{"maximum_thickness_curvature",.01},{"minimum_area_ratio",.99},{"maximum_area_ratio",1.01},
        {"minimum_thickness_ratio",.99},{"maximum_thickness_ratio",1.01},{"maximum_energy_residual_J",1e-10},
        {"relative_energy_residual",.05},{"maximum_native_dt_fraction",.125},{"source_part_id",2000157}}})exact(config,item.first,item.second);
    const auto& velocity=Field(config,"initial_velocity_xyz_m_per_s");
    Require(velocity.IsArray()&&velocity.Size()==3,"Missing frozen initial velocity");
    for(unsigned j=0;j<3;++j)Require(velocity[j].IsNumber()&&Bits(velocity[j].GetDouble())==Bits(j==0?1.:0.),"Changed frozen initial velocity");
    const auto& setup=Field(config,"wall_setup");
    for(const auto& item:std::array<std::pair<const char*,double>,11>{{
        {"declared_leading_gap_m",.0005},{"area_floor_m2",1e-5},{"design_penetration_m",.000375},
        {"penetration_cap_m",.0005},{"kinetic_budget_factor",1.10},{"motion_margin_m",.020},
        {"exposed_clearance_m",1e-6},{"parent_force_error_N",1e-6},{"parent_energy_error_J",1e-9},
        {"maximum_contact_step_rate",.125},{"fixed_dt_s",BaseStep/refinement}}})exact(setup,item.first,item.second);
}
void Run::Open(const std::filesystem::path& path,unsigned ratio) {
    directory=path;refinement=ratio;
    const auto opened=replay.Open(directory);
    if(opened.status!=ReplayStatus::Ok)throw std::runtime_error(opened.diagnostic);
    const auto& info=*replay.info();
    Require(info.kind==ReplayKind::SourcePartWall&&info.horizon_complete&&info.stop_reason.empty()&&
        info.final_epoch==BaseHorizon*ratio&&Bits(info.final_time)==Bits(BaseHorizon*BaseStep),
        "Wall comparison requires three completed full-horizon bundles; prefixes remain partial evidence");
    configuration=ReadDocument(directory/"configuration.json");manifest=ReadDocument(directory/"manifest.json");
    ValidatePilot(configuration,ratio);physical_configuration=Encode(PhysicalConfiguration(configuration));
    placement=ReadBounded(directory/"placed-wall-placement.json",32*1024*1024);
    placed_mesh=ReadBounded(directory/"placed-wall.mesh.json",32*1024*1024);
    original_wall_manifest=ReadBounded(directory/"original-canonical-wall.manifest.json",32*1024*1024);
    manifest_sha256=Sha256(ReadBounded(directory/"manifest.json",32*1024*1024));
    const auto& nodes=Field(configuration,"reference_nodes");Require(nodes.IsArray()&&nodes.Size()==117,"Wrong native source node count");
    long double total_mass=0;
    for(unsigned n=0;n<117;++n) {mass[n]=Real(nodes[n],"mass_kg");Require(mass[n]>0,"Nonpositive native source mass");total_mass+=mass[n];}
    const double speed=configuration["initial_velocity_xyz_m_per_s"][0].GetDouble();initial_momentum=total_mass*speed;
    scales=MakeScales(Real(configuration,"initial_kinetic_J"),static_cast<double>(total_mass),speed,
        Real(Field(configuration,"wall_setup"),"design_penetration_m"));
    events=ReadEvents(*this);
}
Document Run::CommonFrame(std::uint64_t base_epoch) {
    const auto epoch=base_epoch*refinement;
    while(replay.frame()->epoch<epoch) {
        Require(replay.frame()->index+1<replay.info()->frame_count,"Missing common wall sample");
        const auto loaded=replay.Load(replay.frame()->index+1);
        if(loaded.status!=ReplayStatus::Ok)throw std::runtime_error(loaded.diagnostic);
    }
    Require(replay.frame()->epoch==epoch,"Missing common wall sample");
    std::ostringstream name;name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<epoch<<".fields.json";
    auto d=ReadDocument(directory/name.str());
    Require(Unsigned(d,"accepted_epoch")==epoch&&Bits(Real(d,"accepted_time_s"))==Bits(base_epoch*BaseStep),"Changed common wall sample identity");
    maximum_chord=std::max(maximum_chord,Real(d,"maximum_chord_change_m"));
    for(const char* family:{"qeph","t3"}) {
        maximum_strain=std::max(maximum_strain,Real(Field(d,family),"maximum_absolute_strain"));
        maximum_thickness_curvature=std::max(maximum_thickness_curvature,Real(Field(d,family),"maximum_thickness_curvature"));
    }
    CheckFrameMomentum(*this,d);return d;
}
ContactSample Contact(const Value& fields) {
    ContactSample c{Real(fields,"synchronized_kinetic_J"),Real(fields,"total_internal_work_J"),0,
        Real(fields,"cumulative_wall_kick_impulse_N_s"),0};
    const auto& contact=Field(fields,"contact");
    if(!contact.IsNull()) {c.potential=Field(contact,"potential_J")[0].GetDouble();c.wall_reaction=Field(contact,"resultant_N")[0].GetDouble();}
    return c;
}
} // namespace crash::output::wall_comparison
