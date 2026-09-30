#include "SourceAssemblyWallArtifactState.h"
#include "WallArtifactFileIO.h"
#include "case/PlacedCanonicalWall.h"
#include "output/MeshArchive.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <iomanip>
#include <sstream>

namespace crash::output::assembly {
SourceAssemblyWallArtifacts::SourceAssemblyWallArtifacts(const std::string& path,dynamics::SourceAssemblyWallCase& run,
    const WallArchiveRequest& request) {
    wall_fields::CheckCase(run);const auto stamp=run.owner()->accepted();
    Require(!stamp.epoch&&stamp.time==0,"Assembly archive must start at the original physical initial state");
    const auto& source=run.bindings()->source().data();const auto* wall=run.setup()->placed_wall();
    const auto plan=PlanWallArchive(request,source.identity.bytes,wall->source_manifest()->size());
    auto next=std::make_unique<Impl>(path,run,request,plan);next->initial=stamp;
    next->sequence={request,stamp,0,0,plan.frame_capacity,false};
    const auto initialized=next->output.Initialize(*run.owner(),*run.bindings(),
        {stamp.owner_id,request.run_id,request.topology_id},request.asset_id);
    Require(initialized.status==visual::Status::Ok,initialized.message);
    auto config=SourceAssemblyWallConfiguration(run,*next->output.mapping(),request);
    wall_files::JsonBytes(config,WallConfigurationBytes);
    Require(source.authenticated_bytes.size()==source.identity.bytes&&Sha256(source.authenticated_bytes)==source.identity.sha256,
        "Original source inventory bytes changed before output");
    Require(std::filesystem::create_directory(path),"Assembly output directory must be new");
    wall_files::WriteJsonBounded(next->directory/"configuration.json",config,WallConfigurationBytes);next->inventory.Add("configuration.json",WallConfigurationBytes);
    WriteBytes(next->directory/"source-assembly-inventory.json",source.authenticated_bytes);next->inventory.Add("source-assembly-inventory.json",4*1024*1024);
    case_data::WritePlacedCanonicalWallArtifacts(next->directory,*wall);
    next->inventory.Add("original-canonical-wall.manifest.json",WallSmallFileBytes);next->inventory.Add("placed-wall.mesh.json",WallMeshBytes);
    next->inventory.Add("placed-wall.obj",WallObjBytes);next->inventory.Add("placed-wall-placement.json",WallSmallFileBytes);
    next->intervals=std::make_unique<CsvLedgerWriter>(next->directory,WallIntervalHeader,plan.intervals,request.limits.file_bytes);
    next->frames.open(next->directory/"accepted-frames.csv",std::ios::binary);
    next->frames<<std::setprecision(17)<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
    Require(bool(next->frames),"Cannot create assembly accepted frame index");impl_=std::move(next);
}
SourceAssemblyWallArtifacts::~SourceAssemblyWallArtifacts()=default;
void SourceAssemblyWallArtifacts::RecordInterval(const tl::fea::NodalStamp& base,dynamics::SourceAssemblyWallCase& run) {
    auto& s=*impl_;s.Check(run);const auto stamp=run.owner()->accepted();
    s.sequence.CheckInterval(base,stamp);
    const auto row=SourceAssemblyWallInterval(base,run);s.intervals->CheckRow(stamp.epoch,row);
    try {s.intervals->Append(stamp.epoch,row);s.sequence.AcceptInterval(stamp);}catch(...) {s.Abort();throw;}
}
void SourceAssemblyWallArtifacts::WriteFrame(dynamics::SourceAssemblyWallCase& run) {
    auto& s=*impl_;s.Check(run);const auto stamp=run.owner()->accepted();
    const bool prefix=s.sequence.CheckFrame(stamp);
    try {
        const auto capture=run.CaptureAccepted(s.output);Require(bool(capture),capture.message);
        auto fields=SourceAssemblyWallFrameFields(run,s.output);wall_files::JsonBytes(fields,WallFieldBytes);
        const auto mesh=s.output.nodal()->surface().mesh();Require(bool(mesh),"Missing accepted assembly surface");
        const auto n=s.output.nodal()->fields();Require(mesh->GetCoordsVertices().size()==n.node_count,"Assembly mesh lost a physical node");
        for(std::size_t i=0;i<n.node_count;++i)for(unsigned a=0;a<3;++a)
            Require(Bits(mesh->GetCoordsVertices()[i][a])==Bits(n.position_xyz[3*i+a]),"Assembly mesh differs from exact accepted positions");
        std::ostringstream name;name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<stamp.epoch;const auto stem=name.str();
        WriteMeshFiles(s.directory,stem,*mesh);wall_files::WriteJsonBounded(s.directory/(stem+".fields.json"),fields,WallFieldBytes);
        s.inventory.Add(stem+".mesh.json",WallMeshBytes);s.inventory.Add(stem+".obj",WallObjBytes);s.inventory.Add(stem+".fields.json",WallFieldBytes);
        s.frames<<stamp.owner_id<<','<<stamp.epoch<<','<<stamp.time<<','<<stem<<".mesh.json,"<<stem<<".obj\n";
        s.frames.flush();s.intervals->Flush();Require(bool(s.frames),"Assembly frame index flush failed");
        s.sequence.AcceptFrame(stamp,prefix);
    } catch(...) {s.Abort();throw;}
}
} // namespace crash::output::assembly
