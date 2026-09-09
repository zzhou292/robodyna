#include "NormalImpactArtifacts.h"
#include "output/ArtifactIO.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace crash::case_data {
using output::Bits;
using output::Boolean;
using output::Document;
using output::Integer;
using output::Number;
using output::ReadBounded;
using output::Require;
using output::Sha256;
using output::String;
using output::Value;
using output::WriteBytes;
using output::WriteJson;
namespace {
namespace fs=std::filesystem;
constexpr const char* ManifestPin=kCanonicalWallManifestSha256;
Document Metrics(const ImpactMetrics& m) {
    Document doc;doc.SetObject();Integer(doc,"owner_id",m.stamp.owner_id);Integer(doc,"accepted_epoch",m.stamp.epoch);
    Integer(doc,"node_count",m.stamp.node_count);Number(doc,"accepted_time_s",m.stamp.time);Number(doc,"fixed_dt_s",m.stamp.fixed_dt);
    Number(doc,"mass_kg",m.mass);Number(doc,"mean_signed_gap_m",m.mean_gap);Number(doc,"mean_normal_velocity_m_per_s",m.mean_normal_velocity);
    Number(doc,"kinetic_energy_J",m.kinetic_energy);Number(doc,"elastic_energy_J",m.elastic_energy);Number(doc,"wall_impulse_Ns",m.wall_impulse);
    Number(doc,"contact_work_J",m.contact_work);Number(doc,"peak_penetration_m",m.peak_penetration);return doc;
}
} // namespace

struct NormalImpactArtifacts::Impl {
    struct Artifact {std::string name,sha256;std::size_t bytes;};
    fs::path directory;
    NormalImpactConfig config;
    double horizon=0;
    unsigned frame_every=0;
    std::ofstream intervals,frames;
    std::vector<Artifact> artifacts;
    std::uint64_t last_interval_epoch=0,last_frame_epoch=0;
    visual::Identity identity{};
    std::size_t node_count=0;
    double fixed_dt=0,last_interval_time=0;
    bool have_frame=false,finished=false,failed=false;
    explicit Impl(const std::string& path):directory(path){}
    void Inventory(const std::string& name) {
        const auto bytes=ReadBounded(directory/name,32*1024*1024);artifacts.push_back({name,Sha256(bytes),bytes.size()});
    }
    void Mesh(const std::string& stem,const chrono::ChTriangleMeshConnected& source) {
        output::WriteMeshFiles(directory,stem,source);
        Inventory(stem+".mesh.json");Inventory(stem+".obj");
    }
};

NormalImpactArtifacts::NormalImpactArtifacts(const std::string& path,const std::string& canonical_bytes,
    const CanonicalWall& wall,const NormalImpactConfig& config,double horizon,unsigned frame_every):impl_(std::make_unique<Impl>(path)) {
    auto& s=*impl_;s.config=config;s.horizon=horizon;s.frame_every=frame_every;
    Require(wall.loaded()&&Sha256(canonical_bytes)==ManifestPin,"Artifact input is not the verified canonical wall");
    CheckCanonicalWallBinding(wall,canonical_bytes);
    Require(fs::create_directory(s.directory),"Output directory must be new; existing directories are never overwritten");
    WriteCanonicalWallArtifacts(s.directory,wall,canonical_bytes);
    s.Inventory("canonical-wall.manifest.json");
    Document doc;doc.SetObject();String(doc,"schema","tlfea.normal_impact_configuration.v1");String(doc,"status","configured");
    String(doc,"scope","Translational mass-patch normal contact against the finite canonical Yaris wall mesh");
    Boolean(doc,"shell_model",false);Boolean(doc,"vehicle_model",false);Boolean(doc,"general_ccd",false);
    Number(doc,"dt_s",config.dt);Number(doc,"requested_horizon_s",horizon);Number(doc,"areal_density_kg_per_m2",config.areal_density);
    Number(doc,"penalty_per_area_N_per_m3",config.penalty_per_area);Number(doc,"initial_normal_speed_m_per_s",config.speed);
    Number(doc,"max_penetration_m",config.max_penetration);Number(doc,"patch_center_y_m",config.center_y);Number(doc,"patch_center_z_m",config.center_z);
    Number(doc,"patch_width_m",config.width);Integer(doc,"patch_divisions",config.patch_divisions);Boolean(doc,"refine_contact_wall",config.refine_wall);
    Integer(doc,"frame_every_accepted_steps",frame_every);Number(doc,"friction",0);Number(doc,"damping",0);Number(doc,"thickness_offset_m",0);
    Number(doc,"source_wall_friction_not_applied",.6);String(doc,"wall_manifest_sha256",ManifestPin);
    String(doc,"force_phase","F_n evaluated at accepted t_n and applied over [t_n,t_(n+1)]; CSV state is accepted t_(n+1)");
    String(doc,"mesh_archive_precision","Chrono JSON full_precision=true; coordinate bits and topology checked on readback");
    String(doc,"obj_precision","Chrono default visualization precision; use mesh.json/canonical manifest for exact geometry");
    String(doc,"motion_scope","Fixed projected footprint and normal-only motion; admitted penalty overlap is bounded, not general CCD");
    WriteJson(s.directory/"configuration.json",doc);s.Inventory("configuration.json");
    s.Inventory("canonical-wall.mesh.json");s.Inventory("canonical-wall.obj");
    s.intervals.open(s.directory/"accepted-intervals.csv",std::ios::binary);s.frames.open(s.directory/"accepted-frames.csv",std::ios::binary);
    Require(bool(s.intervals)&&bool(s.frames),"Could not create accepted-state CSV files");
    s.intervals<<std::setprecision(std::numeric_limits<double>::max_digits10);
    s.frames<<std::setprecision(std::numeric_limits<double>::max_digits10);
    s.intervals<<"owner_id,base_epoch,attempt,force_eval_time_s,accepted_epoch,accepted_time_s,force_surface_x_N_at_tn,force_surface_y_N_at_tn,force_surface_z_N_at_tn,wall_reaction_x_N_at_tn,wall_reaction_y_N_at_tn,wall_reaction_z_N_at_tn,wall_moment_x_Nm_at_tn,wall_moment_y_Nm_at_tn,wall_moment_z_Nm_at_tn,covered_points_at_tn,active_points_at_tn,elastic_energy_J_at_tn,surface_power_W_at_tn,accepted_mean_gap_m,accepted_normal_velocity_m_per_s,accepted_kinetic_energy_J,accepted_elastic_energy_J,accepted_wall_impulse_Ns,accepted_contact_work_J,accepted_peak_penetration_m\n";
    s.frames<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
}
NormalImpactArtifacts::~NormalImpactArtifacts()=default;
void NormalImpactArtifacts::RecordInterval(const tl::fea::NodalStamp& begin,const ImpactMetrics& m,const tlfea::contact::PlanarContactDiagnostics& d) {
    auto& s=*impl_;Require(!s.finished&&!s.failed&&s.have_frame&&d.valid&&d.owner_id==m.stamp.owner_id&&begin.owner_id==s.identity.owner&&m.stamp.owner_id==s.identity.owner&&
        begin.node_count==s.node_count&&m.stamp.node_count==s.node_count&&begin.fixed_dt==s.fixed_dt&&m.stamp.fixed_dt==s.fixed_dt&&
        d.base_epoch==begin.epoch&&m.stamp.epoch==begin.epoch+1&&begin.epoch==s.last_interval_epoch&&begin.time==s.last_interval_time&&m.stamp.time==begin.time+s.fixed_dt,
        "Interval diagnostics do not belong to this accepted step");
    for(double value:{begin.time,m.stamp.time,d.force_on_surface.x,d.force_on_surface.y,d.force_on_surface.z,
        d.wall_reaction.x,d.wall_reaction.y,d.wall_reaction.z,d.wall_moment.x,d.wall_moment.y,d.wall_moment.z,
        d.elastic_energy,d.surface_power,m.mean_gap,m.mean_normal_velocity,m.kinetic_energy,m.elastic_energy,
        m.wall_impulse,m.contact_work,m.peak_penetration})Require(std::isfinite(value),"Nonfinite accepted CSV value");
    s.intervals<<m.stamp.owner_id<<','<<d.base_epoch<<','<<d.attempt<<','<<begin.time<<','<<m.stamp.epoch<<','<<m.stamp.time<<','
        <<d.force_on_surface.x<<','<<d.force_on_surface.y<<','<<d.force_on_surface.z<<','<<d.wall_reaction.x<<','<<d.wall_reaction.y<<','<<d.wall_reaction.z<<','
        <<d.wall_moment.x<<','<<d.wall_moment.y<<','<<d.wall_moment.z<<','<<d.covered_count<<','<<d.active_count<<','<<d.elastic_energy<<','<<d.surface_power<<','
        <<m.mean_gap<<','<<m.mean_normal_velocity<<','<<m.kinetic_energy<<','<<m.elastic_energy<<','<<m.wall_impulse<<','<<m.contact_work<<','<<m.peak_penetration<<'\n';
    s.intervals.flush();Require(bool(s.intervals),"Accepted interval CSV write failed");s.last_interval_epoch=m.stamp.epoch;s.last_interval_time=m.stamp.time;
}
void NormalImpactArtifacts::WriteAcceptedFrame(const visual::NodalMeshOutput& output,const ImpactMetrics& metrics) {
    auto& s=*impl_;const auto& surface=output.surface();const auto* frame=surface.frame();const auto mesh=surface.mesh();
    Require(!s.finished&&!s.failed&&frame&&mesh&&frame->identity.owner==metrics.stamp.owner_id&&frame->epoch==metrics.stamp.epoch&&frame->time==metrics.stamp.time&&
        (!s.have_frame||frame->epoch>s.last_frame_epoch),"Visible frame does not match the current accepted owner/time");
    if(s.have_frame)Require(frame->identity.owner==s.identity.owner&&frame->identity.run==s.identity.run&&frame->identity.topology==s.identity.topology&&
        metrics.stamp.node_count==s.node_count&&metrics.stamp.fixed_dt==s.fixed_dt,"Frame belongs to a different bound run/topology");
    else Require(frame->epoch==0&&frame->time==0&&metrics.stamp.fixed_dt==s.config.dt,"Artifact run must start at its configured initial accepted state");
    std::ostringstream stem;stem<<"accepted-"<<std::setw(6)<<std::setfill('0')<<frame->epoch;s.Mesh(stem.str(),*mesh);
    s.frames<<metrics.stamp.owner_id<<','<<frame->epoch<<','<<frame->time<<','<<stem.str()<<".mesh.json,"<<stem.str()<<".obj\n";
    s.frames.flush();Require(bool(s.frames),"Accepted frame index write failed");
    s.identity=frame->identity;s.node_count=metrics.stamp.node_count;s.fixed_dt=metrics.stamp.fixed_dt;s.last_frame_epoch=frame->epoch;s.have_frame=true;
}
void NormalImpactArtifacts::Finish(const NormalImpactCase& run,double elapsed_seconds) {
    auto& s=*impl_;const auto* metrics=run.metrics();Require(!s.finished&&!s.failed&&metrics&&s.have_frame&&s.last_frame_epoch==metrics->stamp.epoch&&
        metrics->stamp.owner_id==s.identity.owner&&metrics->stamp.node_count==s.node_count&&metrics->stamp.fixed_dt==s.fixed_dt&&
        s.last_interval_epoch==metrics->stamp.epoch&&std::fabs(metrics->stamp.time-s.horizon)<=128*std::numeric_limits<double>::epsilon()*std::max(1.,s.horizon),
        "Requested accepted horizon or final frame is incomplete");
    const auto* output=run.output();const auto* frame=output?output->surface().frame():nullptr;
    Require(frame&&frame->identity.owner==s.identity.owner&&frame->identity.run==s.identity.run&&frame->identity.topology==s.identity.topology&&
        frame->epoch==metrics->stamp.epoch&&frame->time==metrics->stamp.time,"Final output does not identify the bound run/topology");
    s.intervals.close();s.frames.close();Require(!s.intervals.fail()&&!s.frames.fail(),"CSV close failed");
    s.Inventory("accepted-intervals.csv");s.Inventory("accepted-frames.csv");
    auto final=Metrics(*metrics);String(final,"state","accepted_requested_horizon");
    const double omega=std::sqrt(s.config.penalty_per_area/s.config.areal_density),z=s.config.dt*omega;
    Require(z<2,"Discrete penetration bound is not defined");
    Number(final,"continuous_peak_penetration_reference_m",s.config.speed/omega);
    Number(final,"continuous_contact_duration_reference_s",std::acos(-1.)/omega);
    Number(final,"continuous_wall_impulse_reference_Ns",2*metrics->mass*s.config.speed);
    Number(final,"initial_kinetic_energy_J",.5*metrics->mass*s.config.speed*s.config.speed);
    Number(final,"discrete_penetration_bound_m",s.config.speed/(omega*std::sqrt(1-z*z/4)));
    Number(final,"elapsed_wall_seconds",elapsed_seconds);
    Integer(final,"state_owned_device_bytes",run.state_allocations().device_bytes);Integer(final,"state_owned_device_allocations",run.state_allocations().device_allocations);
    Integer(final,"contact_owned_device_bytes",run.contact_allocations().device_bytes);Integer(final,"contact_owned_device_allocations",run.contact_allocations().device_allocations);
    WriteJson(s.directory/"final-metrics.json",final);s.Inventory("final-metrics.json");
    Document manifest;manifest.SetObject();String(manifest,"schema","tlfea.normal_impact_artifacts.v1");String(manifest,"status","completed");
    String(manifest,"scope","Restricted translational mass-patch normal-contact integration rig");
    Boolean(manifest,"shell_model",false);Boolean(manifest,"vehicle_model",false);Boolean(manifest,"general_ccd",false);
    Integer(manifest,"accepted_epoch",metrics->stamp.epoch);Number(manifest,"accepted_time_s",metrics->stamp.time);String(manifest,"canonical_manifest_sha256",ManifestPin);
    String(manifest,"completion_meaning","Requested horizon committed and accepted artifacts verified; physical regression gates are separate");
    Value inventory(rapidjson::kArrayType);
    for(const auto& artifact:s.artifacts) {
        Value item(rapidjson::kObjectType),name(artifact.name.c_str(),manifest.GetAllocator()),hash(artifact.sha256.c_str(),manifest.GetAllocator());
        item.AddMember("file",name,manifest.GetAllocator());item.AddMember("sha256",hash,manifest.GetAllocator());
        item.AddMember("bytes",Value().SetUint64(artifact.bytes),manifest.GetAllocator());inventory.PushBack(item,manifest.GetAllocator());
    }
    manifest.AddMember("artifacts",inventory,manifest.GetAllocator());WriteJson(s.directory/"manifest.pending.json",manifest);
    Require(!fs::exists(s.directory/"manifest.json"),"Refusing to overwrite successful artifact manifest");
    fs::rename(s.directory/"manifest.pending.json",s.directory/"manifest.json");s.finished=true;
}
void NormalImpactArtifacts::Fail(const char* message,const ImpactMetrics* metrics) noexcept {
    try {
        auto& s=*impl_;if(s.finished||s.failed)return;s.failed=true;s.intervals.flush();s.frames.flush();
        Document failure=metrics?Metrics(*metrics):Document{};if(!metrics)failure.SetObject();
        String(failure,"status","failed");String(failure,"message",message?message:"Unknown failure");
        String(failure,"scope","Partial accepted output only; requested run did not complete");
        WriteJson(s.directory/"failure.json",failure);
    } catch(...) { /* stderr/absent successful manifest still report incomplete output. */ }
}
}  // namespace crash::case_data
