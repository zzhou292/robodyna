#include "SourcePartElasticArtifacts.h"
#include "SourcePartElasticFields.h"
#include "chrono/NodalMeshOutput.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "output/ArtifactInventory.h"
#include "output/SourcePartArtifactSchema.h"
#include "output/CsvLedgerSegments.h"
#include "output/MeshArchive.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace crash::cases::source_part_elastic {
using namespace output;
namespace fs=std::filesystem;
namespace {
const std::string Header=SourcePartIntervalHeader;
bool SameOwner(const tl::fea::NodalStamp& a,const tl::fea::NodalStamp& b) {
    return a.owner_id&&a.owner_id==b.owner_id&&a.node_count==NodeCount&&b.node_count==NodeCount&&a.has_rotations&&b.has_rotations&&
        Bits(a.fixed_dt)==Bits(b.fixed_dt)&&a.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart&&a.temporal_scheme==b.temporal_scheme;
}
}
struct SourcePartElasticArtifacts::Impl {
    fs::path directory; ArtifactInventory inventory; visual::NodalMeshOutput output;
    CsvLedgerPlan plan; std::unique_ptr<CsvLedgerWriter> intervals; std::ofstream frames;
    tl::fea::NodalStamp last; std::uint64_t steps=0,last_frame=0,frame_count=0,frame_cap=0;
    bool failed=false,finished=false;
    explicit Impl(const std::string& path):directory(path),inventory(directory) {}
};
SourcePartElasticArtifacts::SourcePartElasticArtifacts(const std::string& path,SourcePartElasticCase& run,
    std::uint64_t steps,unsigned frame_every,std::uint64_t run_id,std::uint64_t topology_id):impl_(std::make_unique<Impl>(path)) {
    auto& s=*impl_; Require(run.initialized()&&steps&&frame_every,"Source artifacts need an initialized run and positive horizon/cadence");
    Require(run.config().experiment==Experiment::ElasticPulse,"The source pulse archive schema does not support uniform-flight experiments");
    Snapshot initial; const auto captured=run.Capture(&initial); Require(bool(captured),captured.message);
    s.last=initial.stamp; s.steps=steps;
    Require(s.last.epoch==0&&s.last.time==0&&SameOwner(s.last,s.last),"Source output must start at reference epoch zero");
    s.frame_cap=1+steps/frame_every+(steps%frame_every!=0)+(frame_every>1&&steps>1)+1; // Last accepted frame on failed runs.
    Require(s.frame_cap<=kArtifactFrameCap,"Source frame forecast exceeds cap");
    s.plan=PlanCsvLedger("accepted-intervals.csv",Header,steps,27*26);
    Require(s.plan.total_bytes<=kArtifactTotalCap-1024*1024&&
        s.frame_cap<=(kArtifactTotalCap-1024*1024-s.plan.total_bytes)/(256*1024),"Source output forecast exceeds aggregate cap");
    const auto binding=SourcePartSurfaceBinding(run,run_id,topology_id);
    const auto initialized=s.output.Initialize(run.owner(),binding,visual::NodalOutputTiming::StaggeredHalfKick);
    Require(initialized.status==visual::Status::Ok,initialized.message);
    auto config=SourcePartConfiguration(run,binding,steps,frame_every); AppendCsvLedgerSegments(config,&s.plan,1);
    Require(fs::create_directory(s.directory),"Source output directory must be new");
    WriteJson(s.directory/"configuration.json",config); s.inventory.Add("configuration.json");
    s.intervals=std::make_unique<CsvLedgerWriter>(s.directory,Header,s.plan);
    s.frames.open(s.directory/"accepted-frames.csv",std::ios::binary);
    s.frames<<std::setprecision(17)<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
    Require(bool(s.frames),"Cannot create source frame index");
}
SourcePartElasticArtifacts::~SourcePartElasticArtifacts()=default;
void SourcePartElasticArtifacts::RecordInterval(const tl::fea::NodalStamp& base,const tl::fea::NodalStamp& accepted,const Diagnostics& d) {
    auto& s=*impl_; const auto& q=d.shells.qeph; const auto& t=d.shells.t3;
    Require(!s.failed&&!s.finished&&s.frame_count&&SameOwner(s.last,base)&&SameOwner(base,accepted)&&
        base.epoch==s.last.epoch&&Bits(base.time)==Bits(s.last.time)&&accepted.epoch==base.epoch+1&&accepted.epoch<=s.steps&&
        Bits(accepted.time)==Bits(base.time+base.fixed_dt)&&d.shells.valid&&q.valid&&t.valid&&
        q.owner_id==base.owner_id&&t.owner_id==base.owner_id&&q.epoch==accepted.epoch&&t.epoch==accepted.epoch&&
        q.base_epoch==base.epoch&&t.base_epoch==base.epoch&&q.attempt&&q.attempt==t.attempt&&
        q.has_completed_interval&&t.has_completed_interval&&q.accepted_force_assembled&&t.accepted_force_assembled,
        "Source interval is not the next jointly committed state");
    const double values[]{accepted.velocity_time,accepted.reaction_kick_dt,d.external_kick_work,d.external_drift_work,
        d.absolute_external_drift_work,d.synchronized_kinetic,d.total_internal_work,d.energy_residual,d.kinetic_work_residual,
        d.kinetic_work_allowance,d.max_relative_displacement,d.maximum_rotation,
        q.internal_work[0],q.internal_work[1],q.hourglass_viscous_work,t.internal_work[0],t.internal_work[1],
        q.internal_kick_work,t.internal_kick_work,q.internal_drift_work,t.internal_drift_work};
    std::ostringstream row; row<<std::setprecision(17)<<base.owner_id<<','<<base.epoch<<','<<q.attempt<<','<<base.time<<','<<accepted.epoch<<','<<accepted.time;
    for(double value:values) {Require(std::isfinite(value),"Nonfinite source interval diagnostic");row<<','<<value;}
    row<<'\n'; s.intervals->Append(accepted.epoch,row.str()); s.last=accepted;
}
void SourcePartElasticArtifacts::WriteFrame(SourcePartElasticCase& run) {
    auto& s=*impl_; Require(!s.failed&&!s.finished&&s.frame_count<s.frame_cap,"Source output is closed or exceeds its frame forecast");
    Snapshot frame; auto report=run.Capture(&frame); Require(bool(report),report.message);
    Require(SameOwner(s.last,frame.stamp)&&s.last.epoch==frame.stamp.epoch&&Bits(s.last.time)==Bits(frame.stamp.time)&&
        (!s.frame_count||frame.stamp.epoch>s.last_frame),"Source frame is not the recorded accepted endpoint");
    auto fields=SourcePartFrameFields(frame);
    const auto published=s.output.Publish(run.owner()); Require(published.status==visual::Status::Ok,published.message);
    const auto mesh=s.output.surface().mesh(); const auto* stamp=s.output.stamp();
    Require(mesh&&stamp&&stamp->owner_id==frame.stamp.owner_id&&stamp->epoch==frame.stamp.epoch&&Bits(stamp->time)==Bits(frame.stamp.time),
        "Source visible surface and captured fields disagree");
    for(std::size_t n=0;n<NodeCount;++n) for(unsigned axis=0;axis<3;++axis)
        Require(Bits(mesh->GetCoordsVertices()[n][axis])==Bits(frame.position[3*n+axis]),"Source display geometry differs from captured endpoint");
    std::ostringstream name; name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<frame.stamp.epoch; const auto stem=name.str();
    WriteMeshFiles(s.directory,stem,*mesh); WriteJson(s.directory/(stem+".fields.json"),fields);
    s.inventory.Add(stem+".mesh.json",64*1024); s.inventory.Add(stem+".obj",64*1024);
    s.inventory.Add(stem+".fields.json",128*1024);
    s.frames<<frame.stamp.owner_id<<','<<frame.stamp.epoch<<','<<frame.stamp.time<<','<<stem<<".mesh.json,"<<stem<<".obj\n";
    s.frames.flush(); s.intervals->Flush(); Require(bool(s.frames),"Source frame index flush failed");
    s.last_frame=frame.stamp.epoch; ++s.frame_count;
}
void SourcePartElasticArtifacts::Finish(SourcePartElasticCase& run,double elapsed_seconds) {
    auto& s=*impl_; Snapshot final; const auto captured=run.Capture(&final); Require(bool(captured),captured.message);
    Require(!s.failed&&!s.finished&&SameOwner(s.last,final.stamp)&&final.stamp.epoch==s.steps&&s.last.epoch==s.steps&&
        s.last_frame==s.steps&&Bits(final.stamp.time)==Bits(s.last.time)&&std::isfinite(elapsed_seconds)&&elapsed_seconds>=0,
        "Source horizon or final accepted output is incomplete");
    s.intervals->Finish(); s.frames.close(); Require(!s.frames.fail(),"Source frame index close failed");
    for(const auto& segment:s.plan.segments)s.inventory.Add(segment.file);
    s.inventory.Add("accepted-frames.csv");
    Document metrics; metrics.SetObject(); Integer(metrics,"owner_id",final.stamp.owner_id); Integer(metrics,"accepted_epoch",final.stamp.epoch);
    Number(metrics,"accepted_time_s",final.stamp.time); Integer(metrics,"saved_frames",s.frame_count);
    Number(metrics,"elapsed_wall_seconds",elapsed_seconds); Integer(metrics,"owned_device_bytes",run.allocations().device_bytes);
    AppendSourcePartDiagnostics(metrics,final.diagnostics); WriteJson(s.directory/"final-metrics.json",metrics); s.inventory.Add("final-metrics.json");
    Document manifest; manifest.SetObject(); String(manifest,"schema","robo_dyna.source_part_elastic_artifacts.v1");
    String(manifest,"status","completed"); Boolean(manifest,"shell_model",true); Boolean(manifest,"vehicle_model",false);
    Boolean(manifest,"contact",false); String(manifest,"scope","Original Yaris part 2000157, free experimental LAW1 pulse/release; source attachments unapplied");
    Integer(manifest,"accepted_epoch",final.stamp.epoch); Number(manifest,"accepted_time_s",final.stamp.time);
    String(manifest,"completion_meaning","Declared bounded horizon committed; geometry and actual staggered fields archived. Numerical refinement and physical validation are separate gates.");
    s.inventory.AppendTo(manifest); AppendCsvLedgerSegments(manifest,&s.plan,1);
    WriteJson(s.directory/"manifest.pending.json",manifest); Require(!fs::exists(s.directory/"manifest.json"),"Source manifest destination exists");
    fs::rename(s.directory/"manifest.pending.json",s.directory/"manifest.json"); s.finished=true;
}
void SourcePartElasticArtifacts::Fail(const std::string& message) noexcept {
    try {auto& s=*impl_; if(s.failed||s.finished)return; s.failed=true; if(s.intervals)s.intervals->Abort(); s.frames.close();
        Document d;d.SetObject();String(d,"status","failed");String(d,"message",message);Integer(d,"last_recorded_epoch",s.last.epoch);
        Number(d,"last_recorded_time_s",s.last.time);WriteJson(s.directory/"failure.json",d);
    } catch(...) {}
}
} // namespace crash::cases::source_part_elastic
