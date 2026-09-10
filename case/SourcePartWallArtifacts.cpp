#include "SourcePartWallArtifacts.h"
#include "SourcePartWallFields.h"
#include "chrono/NodalMeshOutput.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "output/ArtifactInventory.h"
#include "output/SourcePartWallArtifactSchema.h"
#include "output/CsvLedgerSegments.h"
#include "output/MeshArchive.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace crash::cases::source_part_wall {
using namespace output;
namespace fs=std::filesystem;
namespace {
bool SameOwner(const tl::fea::NodalStamp& a,const tl::fea::NodalStamp& b) {
    return a.owner_id&&a.owner_id==b.owner_id&&a.node_count==source::NodeCount&&b.node_count==source::NodeCount&&a.has_rotations&&b.has_rotations&&
        Bits(a.fixed_dt)==Bits(b.fixed_dt)&&a.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart&&a.temporal_scheme==b.temporal_scheme;
}
}
struct SourcePartWallArtifacts::Impl {
    fs::path directory;ArtifactInventory inventory;visual::NodalMeshOutput output;
    CsvLedgerPlan plan;std::unique_ptr<CsvLedgerWriter> intervals;std::ofstream frames;
    tl::fea::NodalStamp last;std::uint64_t requested_steps=0,last_frame=0,frame_count=0,frame_cap=0;
    bool failed=false,finished=false;
    explicit Impl(const std::string& path):directory(path),inventory(directory){}
};
SourcePartWallArtifacts::SourcePartWallArtifacts(const std::string& path,elastic::SourcePartElasticCase& run,
    std::uint64_t steps,unsigned every,std::uint64_t run_id,std::uint64_t topology_id):impl_(std::make_unique<Impl>(path)) {
    auto& s=*impl_;Require(run.initialized()&&run.wall_setup()&&run.wall_metrics()&&steps&&every&&
        run.config().experiment==elastic::Experiment::MeshWallImpact,"Wall output needs a prepared mesh-wall run and positive horizon/cadence");
    elastic::Snapshot initial;const auto captured=run.Capture(&initial);Require(bool(captured),captured.message);
    CheckAcceptedWallContact(initial.stamp,initial.diagnostics,run.accepted_contact());s.last=initial.stamp;s.requested_steps=steps;
    Require(!s.last.epoch&&s.last.time==0&&SameOwner(s.last,s.last),"Wall output must start at the original reference epoch");
    s.frame_cap=1+steps/every+(steps%every!=0)+(every>1&&steps>1)+1;
    Require(s.frame_cap<=kArtifactFrameCap,"Wall frame forecast exceeds replay cap");
    s.plan=PlanCsvLedger("accepted-intervals.csv",SourcePartWallIntervalHeader,steps,SourcePartWallIntervalColumns*26);
    Require(s.plan.total_bytes<kArtifactTotalCap-1024*1024&&
        s.frame_cap<=(kArtifactTotalCap-1024*1024-s.plan.total_bytes)/SourcePartWallFrameCap,"Wall archive forecast exceeds aggregate cap");
    const auto binding=elastic::SourcePartSurfaceBinding(run,run_id,topology_id);
    const auto initialized=s.output.Initialize(run.owner(),binding,visual::NodalOutputTiming::StaggeredHalfKick);
    Require(initialized.status==visual::Status::Ok,initialized.message);
    auto config=SourcePartWallConfiguration(run,binding,steps,every);AppendCsvLedgerSegments(config,&s.plan,1);
    Require(fs::create_directory(s.directory),"Wall output directory must be new");
    WriteJson(s.directory/"configuration.json",config);s.inventory.Add("configuration.json");
    case_data::WritePlacedCanonicalWallArtifacts(s.directory,*run.wall_setup()->placed_wall());
    for(const char* name:{"original-canonical-wall.manifest.json","placed-wall.mesh.json","placed-wall.obj","placed-wall-placement.json"})s.inventory.Add(name);
    s.intervals=std::make_unique<CsvLedgerWriter>(s.directory,SourcePartWallIntervalHeader,s.plan);
    s.frames.open(s.directory/"accepted-frames.csv",std::ios::binary);
    s.frames<<std::setprecision(17)<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
    Require(bool(s.frames),"Cannot create wall frame index");
}
SourcePartWallArtifacts::~SourcePartWallArtifacts()=default;
void SourcePartWallArtifacts::RecordInterval(const tl::fea::NodalStamp& base,elastic::SourcePartElasticCase& run) {
    auto& s=*impl_;const auto accepted=run.owner().accepted();const auto& d=run.diagnostics();
    CheckAcceptedWallContact(accepted,d,run.accepted_contact());Require(accepted.epoch&&run.accepted_contact()&&run.wall_metrics(),"Missing accepted wall interval/metrics");
    const auto& c=run.accepted_contact()->diagnostics;const auto& m=*run.wall_metrics();
    Require(!s.failed&&!s.finished&&s.frame_count&&SameOwner(s.last,base)&&SameOwner(base,accepted)&&
        base.epoch==s.last.epoch&&Bits(base.time)==Bits(s.last.time)&&accepted.epoch==base.epoch+1&&accepted.epoch<=s.requested_steps&&
        Bits(accepted.time)==Bits(base.time+base.fixed_dt),"Wall interval is not the next committed accepted state");
    const double values[]{accepted.velocity_time,accepted.reaction_kick_dt,d.synchronized_kinetic,d.total_internal_work,
        d.energy_residual,d.kinetic_work_residual,d.kinetic_work_allowance,c.resultant.value,c.resultant.error,c.potential.value,c.potential.error,
        c.kick_work,c.drift_work,c.work_uncertainty,c.quadratic_work_upper,c.wall_kick_impulse,c.wall_kick_impulse_error,
        m.wall_kick_impulse,m.wall_kick_impulse_error,c.maximum_penetration,double(m.active_nodes),m.synchronized_kinetic_uncertainty,
        m.energy_allowance,m.carried_momentum_residual[0],m.carried_momentum_allowance[0],d.maximum_rotation,m.physical_energy_uncertainty,
        double(StrictlySeparatedNodes(run.accepted_contact()))};
    static_assert(sizeof(values)/sizeof(double)+6==SourcePartWallIntervalColumns);
    std::ostringstream row;row<<std::setprecision(17)<<base.owner_id<<','<<base.epoch<<','<<c.attempt<<','<<base.time<<','<<accepted.epoch<<','<<accepted.time;
    for(double value:values){Require(std::isfinite(value),"Nonfinite wall interval diagnostic");row<<','<<value;}
    row<<'\n';s.intervals->Append(accepted.epoch,row.str());s.last=accepted;
}
void SourcePartWallArtifacts::WriteFrame(elastic::SourcePartElasticCase& run) {
    auto& s=*impl_;Require(!s.failed&&!s.finished&&s.frame_count<s.frame_cap,"Wall archive is closed or exceeds its frame cap");
    elastic::Snapshot frame;const auto captured=run.Capture(&frame);Require(bool(captured),captured.message);
    Require(SameOwner(s.last,frame.stamp)&&s.last.epoch==frame.stamp.epoch&&Bits(s.last.time)==Bits(frame.stamp.time)&&
        (!s.frame_count||frame.stamp.epoch>s.last_frame)&&run.wall_setup()&&run.wall_metrics(),"Wall frame is not the recorded accepted endpoint");
    auto fields=SourcePartWallFrameFields(frame,*run.wall_setup(),run.accepted_contact(),*run.wall_metrics());
    const auto published=s.output.Publish(run.owner());Require(published.status==visual::Status::Ok,published.message);
    const auto mesh=s.output.surface().mesh();const auto* stamp=s.output.stamp();
    Require(mesh&&stamp&&stamp->owner_id==frame.stamp.owner_id&&stamp->epoch==frame.stamp.epoch&&Bits(stamp->time)==Bits(frame.stamp.time),
        "Wall frame field/surface association failed");
    for(unsigned n=0;n<source::NodeCount;++n)for(unsigned a=0;a<3;++a)
        Require(Bits(mesh->GetCoordsVertices()[n][a])==Bits(frame.position[3*n+a]),"Visible source mesh differs from accepted fields");
    std::ostringstream name;name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<frame.stamp.epoch;const auto stem=name.str();
    WriteMeshFiles(s.directory,stem,*mesh);WriteJson(s.directory/(stem+".fields.json"),fields);
    s.inventory.Add(stem+".mesh.json",SourcePartWallMeshCap);
    s.inventory.Add(stem+".obj",SourcePartWallObjCap);
    s.inventory.Add(stem+".fields.json",SourcePartWallFieldCap);
    s.frames<<frame.stamp.owner_id<<','<<frame.stamp.epoch<<','<<frame.stamp.time<<','<<stem<<".mesh.json,"<<stem<<".obj\n";
    s.frames.flush();s.intervals->Flush();Require(bool(s.frames),"Wall frame index flush failed");s.last_frame=frame.stamp.epoch;++s.frame_count;
}
void SourcePartWallArtifacts::Finish(elastic::SourcePartElasticCase& run,double elapsed) {Close(run,elapsed,"",false);}
void SourcePartWallArtifacts::FinishPrefix(elastic::SourcePartElasticCase& run,double elapsed,const std::string& reason) {
    Require(!reason.empty(),"Accepted-prefix close requires an explicit stop reason");Close(run,elapsed,reason,true);
}
void SourcePartWallArtifacts::Close(elastic::SourcePartElasticCase& run,double elapsed,const std::string& reason,bool prefix) {
    auto& s=*impl_;elastic::Snapshot final;const auto captured=run.Capture(&final);Require(bool(captured),captured.message);
    Require(!s.failed&&!s.finished&&SameOwner(s.last,final.stamp)&&final.stamp.epoch&&s.last.epoch==final.stamp.epoch&&
        s.last_frame==final.stamp.epoch&&Bits(s.last.time)==Bits(final.stamp.time)&&std::isfinite(elapsed)&&elapsed>=0&&
        (prefix?final.stamp.epoch<s.requested_steps:final.stamp.epoch==s.requested_steps),"Wall accepted output cannot close this horizon/prefix");
    const auto completed=prefix?s.intervals->FinishPrefix():s.plan;if(!prefix)s.intervals->Finish();
    s.frames.close();Require(!s.frames.fail(),"Wall frame index close failed");
    for(const auto& segment:completed.segments)s.inventory.Add(segment.file);s.inventory.Add("accepted-frames.csv");
    Document metrics;metrics.SetObject();Integer(metrics,"owner_id",final.stamp.owner_id);Integer(metrics,"accepted_epoch",final.stamp.epoch);
    Number(metrics,"accepted_time_s",final.stamp.time);Integer(metrics,"saved_frames",s.frame_count);Number(metrics,"elapsed_wall_seconds",elapsed);
    Integer(metrics,"owned_device_bytes",run.allocations().device_bytes);elastic::AppendSourcePartDiagnostics(metrics,final.diagnostics);
    AppendSourcePartWallMetrics(metrics,*run.wall_metrics());Number(metrics,"initial_kinetic_J",run.initial_kinetic_energy());
    WriteJson(s.directory/"final-metrics.json",metrics);s.inventory.Add("final-metrics.json");
    Document manifest;manifest.SetObject();String(manifest,"schema","robo_dyna.source_part_wall_artifacts.v1");String(manifest,"status","completed");
    Boolean(manifest,"shell_model",true);Boolean(manifest,"vehicle_model",false);Boolean(manifest,"contact",true);
    Boolean(manifest,"horizon_complete",!prefix);String(manifest,"stop_reason",reason);
    String(manifest,"scope","Original Yaris part 2000157; experimental elastic mesh-wall impact, source attachments unapplied");
    Integer(manifest,"accepted_epoch",final.stamp.epoch);Number(manifest,"accepted_time_s",final.stamp.time);Integer(manifest,"requested_steps",s.requested_steps);
    String(manifest,"completion_meaning",prefix?"Accepted prefix archived after an explicit stop; requested horizon and rebound are not complete":
        "Declared bounded horizon archived; contact/rebound/refinement and physical validation are separate gates");
    s.inventory.AppendTo(manifest);AppendCsvLedgerSegments(manifest,&completed,1);WriteJson(s.directory/"manifest.pending.json",manifest);
    Require(!fs::exists(s.directory/"manifest.json"),"Wall manifest already exists");fs::rename(s.directory/"manifest.pending.json",s.directory/"manifest.json");s.finished=true;
}
void SourcePartWallArtifacts::Fail(const std::string& message) noexcept {
    try {auto& s=*impl_;if(s.failed||s.finished)return;s.failed=true;if(s.intervals)s.intervals->Abort();s.frames.close();
        Document d;d.SetObject();String(d,"status","failed");String(d,"message",message);Integer(d,"last_recorded_epoch",s.last.epoch);
        Number(d,"last_recorded_time_s",s.last.time);WriteJson(s.directory/"failure.json",d);
    } catch(...){}
}
}
