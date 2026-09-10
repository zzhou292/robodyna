#include "AcceptedReplaySourceAssembly.h"
#include "source_assembly/SourceAssemblyWallSchema.h"
#include <set>
#include <climits>

namespace crash::output::replay_detail {
void ReadSourceAssemblyConfiguration(Bundle& b,const Document& c,const Document& final,const Document& manifest) {
    using namespace output::assembly;
    const auto pinned=source::PinnedYarisSixPartInventory();
    const auto CheckSource=[&](const Value& v) {
        Require(Text(v,"source_inventory_file")=="source-assembly-inventory.json"&&Text(v,"source_inventory_sha256")==pinned.sha256&&
            Unsigned(v,"source_inventory_bytes")==pinned.bytes,"Assembly replay original inventory identity changed");
    };
    Require(Text(c,"schema")==WallConfigurationSchema&&Text(c,"kind")==WallArtifactKind&&Text(manifest,"kind")==WallArtifactKind&&
        Text(final,"kind")==WallArtifactKind&&Text(manifest,"configuration_file")=="configuration.json", "Assembly replay schema association changed");
    CheckSource(c);CheckSource(manifest);
    b.assembly=std::make_shared<AssemblyReplayData>(source::SourceAssembly::ReadBytes(VerifiedBytes(b,"source-assembly-inventory.json"),pinned));
    auto& a=*b.assembly;const auto& s=a.source.data();
    Require(s.nodes.size()==1030&&s.parents.size()==915&&s.qeph_count==804&&s.t3_count==111,"Unsupported pinned assembly extent");
    b.info.node_count=s.nodes.size();b.info.run_id=Unsigned(c,"run_id");b.info.topology_id=Unsigned(c,"topology_id");
    a.instance=Unsigned(c,"source_instance_id");a.asset=Unsigned(c,"asset_id");
    b.source_configuration_id=Unsigned(c,"configuration_id");b.qualification_id=Unsigned(c,"qualification_id");b.wall_binding_id=Unsigned(c,"wall_binding_id");
    Require(b.info.run_id&&b.info.topology_id&&a.instance&&a.asset&&b.source_configuration_id&&b.qualification_id&&b.wall_binding_id,
        "Assembly replay source identifiers are absent");
    for(const auto* v:{&c,&manifest,&final})Require(Unsigned(*v,"owner_id")==b.info.owner_id,"Assembly replay owner changed");
    Require(Unsigned(manifest,"run_id")==b.info.run_id&&Unsigned(manifest,"topology_id")==b.info.topology_id&&
        Unsigned(manifest,"source_instance_id")==a.instance,"Assembly replay manifest source binding changed");
    if(c.HasMember("observe_force_stage")) {WallBool(c,"observe_force_stage",true);a.observe_force_stage=true;}
    WallBool(c,"shell_model",true);WallBool(c,"vehicle_model",false);WallBool(manifest,"contact",true);
    Require(Text(c,"units")=="SI; physical geometry scale 1"&&Text(c,"stress_frame")=="native_corotational_shell_axes",
        "Assembly replay physical scale or stress frame changed");
    a.requested_steps=Unsigned(c,"requested_steps");a.frame_every=Unsigned(c,"frame_every");
    Require(a.requested_steps&&a.requested_steps>=b.info.final_epoch&&a.frame_every&&
        Unsigned(manifest,"requested_steps")==a.requested_steps&&Unsigned(manifest,"frame_every")==a.frame_every&&
        Unsigned(manifest,"saved_frames")==b.entries.size()&&Unsigned(final,"saved_frames")==b.entries.size(),"Assembly replay horizon/frame counts changed");
    Require(Member(manifest,"horizon_complete").IsBool(),"Assembly completion flag has invalid type");
    b.info.horizon_complete=Member(manifest,"horizon_complete").GetBool();b.info.stop_reason=Text(manifest,"stop_reason");
    Require(b.info.stop_reason.size()<=4096&&(b.info.horizon_complete?
        b.info.final_epoch==a.requested_steps&&b.info.stop_reason.empty():b.info.final_epoch<a.requested_steps&&!b.info.stop_reason.empty()),
        "Assembly completed/prefix declaration disagrees");
    CheckReplayTime(Real(c,"requested_horizon_s"),a.requested_steps*b.fixed_dt,b.fixed_dt,a.requested_steps);
    for(std::size_t i=1;i+1<b.entries.size();++i)Require(b.entries[i].epoch==i*a.frame_every,"Assembly frame cadence is incomplete");
    Require(b.entries.size()==2+(b.info.final_epoch-1)/a.frame_every,"Assembly frame cadence or terminal endpoint changed");
    const auto cap=Unsigned(c,"archive_byte_cap"),file_cap=Unsigned(c,"artifact_file_byte_cap"),frames=Unsigned(c,"frame_cap");
    Require(cap&&cap<=b.total_cap&&file_cap&&file_cap<=kFileCap&&frames&&frames<=kFrameCap&&b.entries.size()<=frames&&
        Unsigned(c,"forecast_bytes")<=cap&&Unsigned(c,"forecast_frames")<=frames&&Unsigned(c,"forecast_files")>=b.inventory.size(),
        "Assembly replay explicit output caps changed");
    Require(a.frame_every<=UINT_MAX,"Assembly frame cadence exceeds the writer domain");
    const WallArchiveRequest request{a.requested_steps,b.info.run_id,b.info.topology_id,a.asset,
        static_cast<unsigned>(a.frame_every),{static_cast<std::size_t>(cap),static_cast<std::size_t>(file_cap),static_cast<std::size_t>(frames)}};
    const auto forecast=PlanWallArchive(request,pinned.bytes,b.inventory.at("original-canonical-wall.manifest.json").bytes);
    Require(Unsigned(c,"forecast_bytes")==forecast.forecast_bytes&&Unsigned(c,"forecast_frames")==forecast.frame_capacity&&
        Unsigned(c,"forecast_files")==forecast.forecast_files,"Assembly forecast differs from its declared shared plan");
    Require(b.manifest_bytes<=cap,"Assembly manifest exceeds configured total cap");
    std::size_t bytes=b.manifest_bytes;for(const auto& item:b.inventory) {
        Require(item.second.bytes<=file_cap&&item.second.bytes<=cap-bytes,"Assembly inventory exceeds configured cap");bytes+=item.second.bytes;
    }
    const auto& limits=Member(c,"deformation_limits");Require(limits.IsObject(),"Assembly deformation limits missing");
    for(const char* key:{"maximum_displacement_m","maximum_rotation_rad","maximum_rotation_increment_rad","maximum_strain",
        "maximum_thickness_curvature","minimum_area_ratio","maximum_area_ratio","minimum_thickness_ratio",
        "maximum_thickness_ratio","maximum_native_dt_fraction"})Require(Real(limits,key)>0,"Assembly deformation limit is invalid");
    a.minimum_thickness_ratio=Real(limits,"minimum_thickness_ratio");a.maximum_thickness_ratio=Real(limits,"maximum_thickness_ratio");
    a.maximum_area_ratio=Real(limits,"maximum_area_ratio");a.maximum_rotation=Real(limits,"maximum_rotation_rad");
    a.maximum_displacement=Real(limits,"maximum_displacement_m");
    Require(a.minimum_thickness_ratio<=1&&a.maximum_thickness_ratio>=1&&a.maximum_area_ratio>=1,"Assembly deformation envelope changed");
    const auto& storage=Member(c,"storage_limits");Require(storage.IsObject(),"Assembly storage declarations missing");
    for(const char* key:{"max_nodes","max_parents","max_host_bytes","max_device_bytes","owner_device_bytes","qeph_device_bytes",
        "t3_device_bytes","publication_max_nodes","publication_device_bytes","publication_host_bytes","contact_max_parents",
        "contact_max_nodes","contact_max_global_nodes","contact_device_bytes","contact_host_bytes"})
        Require(Unsigned(storage,key)>0,"Assembly storage declaration is invalid");
    Require(Unsigned(storage,"max_nodes")>=s.nodes.size()&&Unsigned(storage,"max_nodes")<=2048&&
        Unsigned(storage,"max_parents")>=s.parents.size()&&Unsigned(storage,"max_parents")<=1024,
        "Assembly active storage source extent changed");
    CheckAssemblySurface(b,c);
    for(const auto& row:Member(c,"triangle_binding").GetArray()) {
        std::array<std::uint64_t,9> ids{};for(unsigned j=0;j<9;++j)ids[j]=AssemblyId(row[j]);
        b.source_triangles.push_back(ids);b.topology.push_back({int(ids[0]),int(ids[1]),int(ids[2])});
    }
    ReadAssemblyDeclarations(b,Member(c,"input"));ReadAssemblyGroups(b,Member(c,"input"));
    ReadAssemblyWallSetup(b,Member(c,"wall_setup"));ReadAssemblyIntervals(b,c,manifest);
    std::set<std::string> expected_files{"configuration.json","source-assembly-inventory.json","accepted-frames.csv","final-metrics.json",
        "original-canonical-wall.manifest.json","placed-wall.mesh.json","placed-wall.obj","placed-wall-placement.json"};
    for(const auto& file:a.interval_files)Require(expected_files.insert(file).second,"Assembly duplicate ledger file association");
    for(const auto& entry:b.entries)for(const auto& file:{entry.mesh,entry.obj,entry.mesh.substr(0,entry.mesh.size()-10)+".fields.json"})
        Require(expected_files.insert(file).second,"Assembly duplicate frame file association");
    Require(expected_files.size()==b.inventory.size(),"Assembly archive contains undeclared files");
    for(const auto& file:expected_files)Require(b.inventory.count(file),"Assembly archive omits a required file");
    auto info=std::make_shared<ReplayAssemblyInfo>();info->source_instance_id=a.instance;info->inventory_sha256=s.identity.sha256;
    info->inventory_bytes=s.identity.bytes;info->boundary_policy=s.boundary.policy;info->parents=s.parents.size();
    info->observe_force_stage=a.observe_force_stage;
    info->qeph=s.qeph_count;info->t3=s.t3_count;info->groups=a.group_count;info->members=a.member_count;
    for(const auto& p:s.parts)info->part_ids.push_back(p.id);
    for(const auto& m:s.materials)info->material_ids.push_back(m.id);
    for(const auto& x:s.sections)info->section_ids.push_back(x.id);
    for(const auto& curve:s.curves)info->curve_ids.push_back(curve.id);
    b.info.source_assembly=std::move(info);
    a.final_diagnostics.CopyFrom(Member(final,"diagnostics"),a.final_diagnostics.GetAllocator());
    a.maximum_plastic=Real(a.final_diagnostics,"maximum_plastic_strain");
    Require(a.maximum_plastic>=0,"Assembly final plastic strain is invalid");
    CheckAssemblyStamp(b,b.entries.back(),Member(final,"stamp"));
    CheckAssemblyDiagnostics(b,b.entries.back(),a.final_diagnostics,nullptr);
}
} // namespace crash::output::replay_detail
