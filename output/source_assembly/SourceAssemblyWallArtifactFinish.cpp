#include "SourceAssemblyWallArtifactState.h"
#include "WallArtifactFileIO.h"
#include "WallFieldValues.h"
#include <cmath>

namespace crash::output::assembly {
void SourceAssemblyWallArtifacts::Finish(dynamics::SourceAssemblyWallCase& run,double elapsed) {Close(run,elapsed,"",false);}
void SourceAssemblyWallArtifacts::FinishPrefix(dynamics::SourceAssemblyWallCase& run,double elapsed,const std::string& reason) {
    Require(!reason.empty()&&reason.size()<=4096,"Accepted assembly prefix requires a bounded explicit stop reason");Close(run,elapsed,reason,true);
}
void SourceAssemblyWallArtifacts::Close(dynamics::SourceAssemblyWallCase& run,double elapsed,const std::string& reason,bool prefix) {
    auto& s=*impl_;s.Check(run);const auto stamp=run.owner()->accepted();
    s.sequence.CheckClose(stamp,prefix);Require(std::isfinite(elapsed)&&elapsed>=0,"Invalid assembly archive elapsed time");
    try {
        auto completed=prefix?s.intervals->FinishPrefix():s.plan.intervals;if(!prefix)s.intervals->Finish();
        s.frames.close();Require(!s.frames.fail(),"Assembly frame index close failed");
        for(const auto& segment:completed.segments)s.inventory.Add(segment.file,s.request.limits.file_bytes);
        s.inventory.Add("accepted-frames.csv",WallFrameIndexBytes);
        Document metrics;metrics.SetObject();String(metrics,"kind",WallArtifactKind);Integer(metrics,"owner_id",stamp.owner_id);
        Integer(metrics,"accepted_epoch",stamp.epoch);Number(metrics,"accepted_time_s",stamp.time);Integer(metrics,"saved_frames",s.sequence.frame_count);
        Number(metrics,"elapsed_wall_seconds",elapsed);Integer(metrics,"owned_device_bytes",run.allocations().device_bytes);
        Integer(metrics,"owned_device_allocations",run.allocations().device_allocations);Integer(metrics,"engine_host_payload_bytes",run.host_payload_bytes());
        wall_fields::Child(metrics,"stamp",wall_fields::StampDocument(stamp));wall_fields::Child(metrics,"diagnostics",wall_fields::DiagnosticsDocument(*run.diagnostics()));
        wall_files::WriteJsonBounded(s.directory/"final-metrics.json",metrics,WallSmallFileBytes);s.inventory.Add("final-metrics.json",WallSmallFileBytes);
        Document d;d.SetObject();String(d,"schema",WallArtifactSchema);String(d,"kind",WallArtifactKind);String(d,"status","completed");
        Boolean(d,"shell_model",true);Boolean(d,"vehicle_model",false);Boolean(d,"contact",true);Boolean(d,"horizon_complete",!prefix);String(d,"stop_reason",reason);
        String(d,"scope","Original connected six-part Yaris component; complete internal groups and explicit released external connections");
        String(d,"completion_meaning",prefix?"Accepted prefix archived after an explicit stop; requested horizon is incomplete":
            "Declared bounded accepted horizon archived; visible deformation, collocated physical energy and full vehicle validation remain separate gates");
        Integer(d,"owner_id",stamp.owner_id);Integer(d,"run_id",s.request.run_id);Integer(d,"topology_id",s.request.topology_id);
        Integer(d,"source_instance_id",run.bindings()->source_instance_id());
        String(d,"source_inventory_file","source-assembly-inventory.json");String(d,"source_inventory_sha256",run.bindings()->source().data().identity.sha256);
        Integer(d,"source_inventory_bytes",run.bindings()->source().data().identity.bytes);String(d,"configuration_file","configuration.json");
        Integer(d,"accepted_epoch",stamp.epoch);Number(d,"accepted_time_s",stamp.time);Integer(d,"requested_steps",s.request.steps);
        Integer(d,"frame_every",s.request.frame_every);Integer(d,"saved_frames",s.sequence.frame_count);
        s.inventory.AppendTo(d);AppendCsvLedgerSegments(d,&completed,1);wall_files::PublishManifest(s.directory,d,s.request.limits.total_bytes);s.finished=true;
    } catch(...) {s.Abort();throw;}
}
void SourceAssemblyWallArtifacts::Fail(const std::string& message) noexcept {
    try {if(!impl_||impl_->finished)return;auto& s=*impl_;s.Abort();
        if(std::filesystem::exists(s.directory/"failure.json"))return;
        Document d;d.SetObject();String(d,"kind",WallArtifactKind);String(d,"status","failed");String(d,"message",message.substr(0,4096));
        Integer(d,"last_recorded_epoch",s.sequence.last.epoch);Number(d,"last_recorded_time_s",s.sequence.last.time);Integer(d,"saved_frames",s.sequence.frame_count);
        wall_files::WriteJsonBounded(s.directory/"failure.json",d,WallSmallFileBytes);
    }catch(...) {}
}
} // namespace crash::output::assembly
