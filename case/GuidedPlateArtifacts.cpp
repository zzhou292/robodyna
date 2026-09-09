#include "GuidedPlateArtifacts.h"
#include "GuidedPlateContactIdentity.h"
#include "output/ContactIntegrationMetadata.h"
#include "GuidedPlateFields.h"
#include "GuidedPlateIntervals.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactInventory.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace crash::case_data {
namespace fs=std::filesystem;
using namespace crash::output;
namespace ct=tlfea::contact;
namespace {
bool SameOwner(const tl::fea::NodalStamp& a,const tl::fea::NodalStamp& b) {
    return a.owner_id&&a.owner_id==b.owner_id&&a.node_count==b.node_count&&a.has_rotations&&b.has_rotations&&
           Bits(a.fixed_dt)==Bits(b.fixed_dt);
}
void CheckBoundWall(const GuidedPlateCase& run,const CanonicalWall& wall) {
    const auto view=run.wall_mesh();
    Require(view.vertices&&view.triangles&&view.vertex_count==wall.vertices().size()&&view.triangle_count==wall.triangles().size(),
            "Guided run does not retain the archived canonical wall");
    for(unsigned i=0;i<view.vertex_count;++i) {
        const auto& a=view.vertices[i]; const auto& b=wall.vertices()[i];
        Require(a.source_node_id==b.source_node_id&&a.assembled_source_node_id==b.assembled_source_node_id&&
            Bits(a.position.x)==Bits(b.position_m[0])&&Bits(a.position.y)==Bits(b.position_m[1])&&Bits(a.position.z)==Bits(b.position_m[2]),
            "Guided run wall vertex differs from authenticated source");
    }
    for(unsigned i=0;i<view.triangle_count;++i) {
        const auto& a=view.triangles[i]; const auto& b=wall.triangles()[i];
        Require(a.triangle_id==b.triangle_id&&a.source_quad_id==b.source_quad_id&&a.assembled_source_quad_id==b.assembled_source_quad_id,
                "Guided run wall triangle source differs from authenticated input");
        for(unsigned n=0;n<3;++n)Require(a.nodes[n]==b.vertex_indices[n],"Guided run wall connectivity differs from authenticated input");
    }
}
void CheckInterval(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& m,ct::Q4PlanarIntegrationBackend backend,std::uint64_t qualification) {
    const auto& s=m.shell; const auto& c=m.contact; const auto& applied=m.applied_contact;
    Require(SameOwner(base,m.stamp)&&std::isfinite(base.time)&&base.time>=0&&m.stamp.epoch==base.epoch+1&&
        m.stamp.time==base.time+base.fixed_dt&&m.stamp.time>base.time&&m.stamp.reactions_valid&&
        m.stamp.reaction_base_epoch==base.epoch&&m.stamp.reaction_time==base.time&&s.valid&&c.valid&&applied.valid&&
        s.owner_id==base.owner_id&&c.owner_id==base.owner_id&&applied.owner_id==base.owner_id&&
        s.base_epoch==base.epoch&&c.base_epoch==base.epoch&&applied.base_epoch==base.epoch&&
        s.attempt&&s.attempt==c.attempt&&s.attempt==applied.attempt&&
        qualification&&s.configuration_id==qualification&&c.configuration_id==s.configuration_id&&applied.configuration_id==s.configuration_id&&
        c.wall_binding_id==kGuidedPlateWallBinding&&applied.wall_binding_id==c.wall_binding_id&&
        s.phase==tl::fea::reissner::ShellBatchPhase::kPreparedCandidate&&c.phase==ct::Q4PlanarContactPhase::PreparedCandidate&&
        applied.phase==ct::Q4PlanarContactPhase::AcceptedBase&&ValidGuidedContactPartition(c,backend)&&
        ValidGuidedContactPartition(applied,backend),"Guided output is not a matching committed two-contributor interval");
}
Document FinalMetrics(const GuidedPlateCase& run,std::size_t frame_count,double elapsed) {
    const auto& m=*run.metrics(); Document d; d.SetObject();
    String(d,guided_experiment_metadata::Field,GuidedExperimentName(run.experiment()));
    Integer(d,"qualification_id",run.guided_data()->qualification_id);
    String(d,contact_metadata::BackendField,GuidedContactBackendName(run.integration_backend()));
    Integer(d,"owner_id",m.stamp.owner_id); Integer(d,"accepted_epoch",m.stamp.epoch); Number(d,"accepted_time_s",m.stamp.time);
    Integer(d,"saved_frames",frame_count); Number(d,"elapsed_wall_seconds",elapsed); Number(d,"initial_energy_J",m.initial_energy);
    Number(d,"final_shell_elastic_energy_J",m.shell.elastic_energy); Number(d,"final_contact_potential_J",m.contact.potential.value);
    Number(d,"final_contact_potential_error_J",m.contact.potential.error); Number(d,"final_kinetic_energy_J",m.work.kinetic_energy);
    Number(d,"final_kinetic_translation_J",m.shell.kinetic_translation); Number(d,"final_kinetic_physical_rotation_J",m.shell.kinetic_physical_rotation);
    Number(d,"final_kinetic_artificial_drilling_J",m.shell.kinetic_artificial_drilling); Number(d,"maximum_relative_energy_error",m.maximum_relative_energy_error);
    String(d,"maximum_relative_energy_error_scope","Maximum numerical-value error; per-step admission also includes contact potential uncertainty");
    Number(d,"peak_penetration_m",m.peak_penetration); Number(d,"wall_impulse_x_Ns",m.wall_impulse.x);
    Number(d,"wall_moment_impulse_y_Nms",m.wall_moment_impulse.y); Number(d,"wall_moment_impulse_z_Nms",m.wall_moment_impulse.z);
    Number(d,"shell_midpoint_work_J",m.shell_midpoint_work); Number(d,"contact_midpoint_work_J",m.contact_midpoint_work);
    Number(d,"shell_coordinate_work_J",m.shell_coordinate_work); Number(d,"contact_coordinate_work_J",m.contact_coordinate_work);
    Integer(d,"last_operator_epoch",m.last_operator_epoch); Number(d,"last_operator_norm_per_s2",m.last_operator_norm);
    Integer(d,"full_state_audit_reads",m.full_state_audit_reads);
    Integer(d,"state_owned_device_bytes",run.state_allocations().device_bytes); Integer(d,"state_owned_device_allocations",run.state_allocations().device_allocations);
    Integer(d,"element_owned_device_bytes",run.element_allocations().device_bytes); Integer(d,"element_owned_device_allocations",run.element_allocations().device_allocations);
    Integer(d,"contact_owned_device_bytes",run.contact_allocations().device_bytes); Integer(d,"contact_owned_device_allocations",run.contact_allocations().device_allocations);
    return d;
}
} // namespace
struct GuidedPlateArtifacts::Impl {
    fs::path directory; ArtifactInventory inventory;
    std::array<std::ofstream,3> intervals; std::ofstream frames;
    std::array<std::size_t,3> ledger_bytes{};
    GuidedPlateOutputForecast forecast;
    tl::fea::NodalStamp last_interval;
    visual::Identity identity;
    std::uint64_t required_steps=0,last_frame_epoch=0,frame_count=0;
    double horizon=0; std::size_t static_bytes=0;
    bool failed=false,finished=false;
    ct::Q4PlanarIntegrationBackend backend=ct::Q4PlanarIntegrationBackend::ScalarDyadicSquares;
    reference::GuidedPlateExperiment experiment=reference::GuidedPlateExperiment::Original;
    std::uint64_t qualification=0;
    explicit Impl(const std::string& path):directory(path),inventory(directory) {}
    void CheckRun(const GuidedPlateCase& run) const {
        Require(run.metrics()&&run.modal()&&run.guided_data()&&run.output()&&run.output()->surface().binding(),"Missing initialized guided output run");
        const auto& b=run.output()->surface().binding()->identity;
        Require(SameOwner(last_interval,run.metrics()->stamp)&&b.owner==identity.owner&&b.run==identity.run&&b.topology==identity.topology&&
            run.metrics()->required_steps==required_steps&&run.modal()->horizon==horizon&&run.guided_data()->wall_binding_id==kGuidedPlateWallBinding&&
            run.integration_backend()==backend&&ValidGuidedContactPartition(run.metrics()->contact,backend)&&
            run.experiment()==experiment&&GuidedExperimentIdentity(experiment,qualification)&&
            run.guided_data()->qualification_id==qualification&&run.metrics()->shell.configuration_id==qualification&&
            run.metrics()->contact.configuration_id==qualification,
            "Guided output belongs to a different owner/run/topology");
    }
    void AddStatic(const std::string& file) {
        const auto before=inventory.bytes(); inventory.Add(file,1024*1024);
        static_bytes+=inventory.bytes()-before; Require(static_bytes<=kGuidedStaticReserve,"Guided static artifact reserve exceeded");
    }
    void Close() noexcept { for(auto& f:intervals)f.close(); frames.close(); }
};

GuidedPlateArtifacts::GuidedPlateArtifacts(const std::string& path,const std::string& bytes,const CanonicalWall& wall,
                                         const GuidedPlateCase& run,unsigned frame_every):impl_(std::make_unique<Impl>(path)) {
    auto& s=*impl_;
    Require(run.metrics()&&run.modal()&&run.output()&&run.output()->surface().binding(),"Guided artifacts require an initialized run");
    s.last_interval=run.metrics()->stamp; s.required_steps=run.metrics()->required_steps; s.horizon=run.modal()->horizon;
    s.identity=run.output()->surface().binding()->identity;
    s.backend=run.integration_backend();
    Require(run.guided_data(),"Guided experiment metadata is unavailable");
    s.experiment=run.experiment();s.qualification=run.guided_data()->qualification_id;
    Require(s.last_interval.owner_id&&s.last_interval.epoch==0&&s.last_interval.time==0&&s.last_interval.fixed_dt>0&&
            std::isfinite(s.last_interval.fixed_dt)&&std::isfinite(s.horizon)&&s.horizon>0,"Guided output must start at its initial accepted state");
    s.CheckRun(run); CheckCanonicalWallBinding(wall,bytes); CheckBoundWall(run,wall);
    s.forecast=ForecastGuidedPlateOutput(s.required_steps,frame_every);
    auto configuration=GuidedPlateConfiguration(run,frame_every,kCanonicalWallManifestSha256);
    Integer(configuration,"forecast_total_bytes",s.forecast.total_bytes); Integer(configuration,"forecast_frames",s.forecast.frames);
    Integer(configuration,"per_file_cap_bytes",kArtifactFileCap); Integer(configuration,"aggregate_cap_bytes",kArtifactTotalCap);
    // All input/source/size admission above precedes the first filesystem write.
    Require(fs::create_directory(s.directory),"Guided output directory must be new; existing output is preserved");
    WriteJson(s.directory/"configuration.json",configuration); s.AddStatic("configuration.json");
    WriteCanonicalWallArtifacts(s.directory,wall,bytes);
    for(const char* file:{"canonical-wall.manifest.json","canonical-wall.mesh.json","canonical-wall.obj"})s.AddStatic(file);
    const auto& headers=GuidedPlateIntervalHeaders();
    for(unsigned n=0;n<3;++n) {
        s.intervals[n].open(s.directory/kGuidedIntervalFiles[n],std::ios::binary);
        Require(bool(s.intervals[n]),"Could not create guided contributor ledger");
        s.intervals[n]<<headers[n]; Require(bool(s.intervals[n]),"Could not write guided ledger header"); s.ledger_bytes[n]=headers[n].size();
    }
    s.frames.open(s.directory/"accepted-frames.csv",std::ios::binary); Require(bool(s.frames),"Could not create guided accepted frame index");
    s.frames<<std::setprecision(17)<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
    Require(bool(s.frames),"Could not write guided frame header");
}
GuidedPlateArtifacts::~GuidedPlateArtifacts()=default;
void GuidedPlateArtifacts::RecordInterval(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& m) {
    auto& s=*impl_;
    Require(!s.failed&&!s.finished&&s.frame_count&&SameOwner(s.last_interval,base)&&
        base.epoch==s.last_interval.epoch&&Bits(base.time)==Bits(s.last_interval.time)&&m.stamp.epoch<=s.required_steps&&
        m.required_steps==s.required_steps,
        "Guided output interval is stale, skipped or closed");
    CheckInterval(base,m,s.backend,s.qualification); const auto rows=GuidedPlateIntervalRows(base,m);
    for(unsigned n=0;n<3;++n)Require(s.ledger_bytes[n]<=s.forecast.ledger_bytes[n]&&
        rows[n].size()<=s.forecast.ledger_bytes[n]-s.ledger_bytes[n],"Guided ledger exceeds admitted byte forecast");
    try {
        for(unsigned n=0;n<3;++n) {s.intervals[n]<<rows[n]; Require(bool(s.intervals[n]),"Guided interval write failed");}
        for(unsigned n=0;n<3;++n)s.ledger_bytes[n]+=rows[n].size(); s.last_interval=m.stamp;
    } catch(...) {Fail("Guided accepted interval output failed"); throw;}
}
void GuidedPlateArtifacts::WriteFrame(GuidedPlateCase& run) {
    auto& s=*impl_; Require(!s.failed&&!s.finished&&s.frame_count<s.forecast.frames,"Guided output is closed or exceeds admitted frame capacity");
    s.CheckRun(run);
    Require(run.metrics()->stamp.epoch==s.last_interval.epoch&&Bits(run.metrics()->stamp.time)==Bits(s.last_interval.time)&&
        (!s.frame_count||run.metrics()->stamp.epoch>s.last_frame_epoch),"Guided frame is not the next recorded accepted endpoint");
    GuidedPlateFrame frame; const auto report=run.Capture(frame); Require(report.status==GuidedPlateStatus::Ok,report.diagnostic.c_str());
    auto fields=GuidedPlateFrameFields(frame,*run.guided_data());
    const auto* visible=run.output()->surface().frame(); const auto mesh=run.output()->surface().mesh();
    Require(visible&&mesh&&visible->identity.owner==s.identity.owner&&visible->identity.run==s.identity.run&&visible->identity.topology==s.identity.topology&&
        visible->epoch==frame.stamp.epoch&&Bits(visible->time)==Bits(frame.stamp.time)&&mesh->GetCoordsVertices().size()==reference::kCouponNodes,
        "Guided visible mesh is not the captured accepted configuration");
    for(unsigned n=0;n<reference::kCouponNodes;++n)for(unsigned axis=0;axis<3;++axis)
        Require(Bits(mesh->GetCoordsVertices()[n][axis])==Bits(frame.position[3*n+axis]),"Guided mesh and captured fields differ");
    std::ostringstream name; name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<frame.stamp.epoch; const auto stem=name.str();
    try {
        WriteMeshFiles(s.directory,stem,*mesh); WriteJson(s.directory/(stem+".fields.json"),fields);
        s.inventory.Add(stem+".mesh.json",kGuidedMeshFileCap); s.inventory.Add(stem+".obj",kGuidedObjFileCap);
        s.inventory.Add(stem+".fields.json",kGuidedFieldFileCap);
        s.frames<<frame.stamp.owner_id<<','<<frame.stamp.epoch<<','<<frame.stamp.time<<','<<stem<<".mesh.json,"<<stem<<".obj\n";
        s.frames.flush(); Require(bool(s.frames),"Guided accepted frame index failed");
        for(auto& ledger:s.intervals) {ledger.flush(); Require(bool(ledger),"Guided ledger flush failed");}
        s.last_frame_epoch=frame.stamp.epoch; ++s.frame_count;
    } catch(...) {Fail("Guided accepted frame output failed"); throw;}
}
void GuidedPlateArtifacts::Finish(const GuidedPlateCase& run,double elapsed) {
    auto& s=*impl_; Require(!s.failed&&!s.finished,"Guided output is already closed"); s.CheckRun(run);
    const auto& m=*run.metrics();
    const double roundoff=(static_cast<double>(s.required_steps)+1)*std::numeric_limits<double>::epsilon();
    Require(std::isfinite(elapsed)&&elapsed>=0&&m.stamp.epoch==s.required_steps&&s.last_interval.epoch==s.required_steps&&
        Bits(s.last_interval.time)==Bits(m.stamp.time)&&s.frame_count&&s.last_frame_epoch==s.required_steps&&roundoff<1e-3&&
        std::abs(m.stamp.time-s.horizon)<=8*roundoff*std::max(s.horizon,m.stamp.time),"Guided full admitted horizon or final frame is incomplete");
    auto final=FinalMetrics(run,s.frame_count,elapsed);
    try {
        s.Close(); for(const auto& f:s.intervals)Require(!f.fail(),"Guided ledger close failed"); Require(!s.frames.fail(),"Guided frame index close failed");
        for(unsigned n=0;n<3;++n)s.inventory.Add(kGuidedIntervalFiles[n],s.forecast.ledger_bytes[n]);
        s.AddStatic("accepted-frames.csv"); WriteJson(s.directory/"final-metrics.json",final); s.AddStatic("final-metrics.json");
        Require(s.inventory.bytes()<=s.forecast.total_bytes,"Guided completed output exceeds admitted aggregate forecast");
        Document manifest; manifest.SetObject(); String(manifest,"schema","robo_dyna.guided_plate_artifacts.v1"); String(manifest,"status","completed");
        String(manifest,"scope","Synthetic guided elastic two-Q4 plate against the original canonical mesh wall");
        String(manifest,contact_metadata::BackendField,GuidedContactBackendName(s.backend));
        String(manifest,guided_experiment_metadata::Field,GuidedExperimentName(s.experiment));
        Integer(manifest,"qualification_id",s.qualification);
        Boolean(manifest,"shell_model",true); Boolean(manifest,"vehicle_model",false); Boolean(manifest,"contact",true);
        Integer(manifest,"owner_id",m.stamp.owner_id); Integer(manifest,"accepted_epoch",m.stamp.epoch); Number(manifest,"accepted_time_s",m.stamp.time);
        String(manifest,"completion_meaning","Admitted horizon committed and accepted artifacts verified; refinement and inspected rendering are separate gates");
        s.inventory.AppendTo(manifest); WriteJson(s.directory/"manifest.pending.json",manifest);
        Require(!fs::exists(s.directory/"manifest.json"),"Refusing to overwrite completed guided manifest");
        fs::rename(s.directory/"manifest.pending.json",s.directory/"manifest.json"); s.finished=true;
    } catch(...) {Fail("Guided output completion failed"); throw;}
}
void GuidedPlateArtifacts::Fail(const std::string& diagnostic) noexcept {
    try {
        auto& s=*impl_; if(s.failed||s.finished)return; s.failed=true; s.Close();
        Document failure; failure.SetObject(); String(failure,"status","failed"); String(failure,"message",diagnostic);
        Integer(failure,"last_recorded_epoch",s.last_interval.epoch); Number(failure,"last_recorded_time_s",s.last_interval.time);
        WriteJson(s.directory/"failure.json",failure);
    } catch(...) { /* The absence of a completed manifest still records incomplete output. */ }
}
} // namespace crash::case_data
