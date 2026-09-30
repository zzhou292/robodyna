#include "RunState.h"
#include "output/physical_frames/Mapping.h"
#include "case/vehicle_wall/Artifacts.h"
namespace crash::output::physical_run {
namespace {
void CheckSetup(const cases::vehicle_wall::VehicleWallSetup& setup,const physical_frames::Mapping& mapping,
    const records::Context& context,const records::source::BundleRequest& request,Profile profile) {
    const auto& execution=mapping.execution();
    Require(setup.execution().model().SharesStorage(execution.model()) &&
        setup.execution().execution().parents().data()==execution.execution().parents().data() &&
        setup.execution().physical().failure()->parent(0)==execution.physical().failure()->parent(0),
        "Physical wall archive belongs to another actual execution backing");
    Require(context.identity().source_mapping_sha256==mapping.source_mapping().digest() &&
        context.identity().source_instance==execution.physical().domain()->source_instance_id() &&
        Bits(request.archive.requested_duration)==Bits(setup.settings().requested_duration_s),
        "Physical wall archive duration/source differs from selected setup");
    Require(profile.beam18==bool(execution.model().structural_beams()),
        "Physical wall archive observation omits or invents structural beams");
    const auto original=setup.wall().view(),selected=setup.selected_wall_view();
    Require(original.vertex_count==62 && original.triangle_count==100,
        "Physical wall archive requires the complete pinned original wall");
    const bool placed=setup.settings().mesh_profile==cases::vehicle_wall::WallMeshProfile::PlacedOriginal;
    Require(selected.vertex_count==(placed?62u:4u) && selected.triangle_count==(placed?100u:2u),
        "Physical wall serializer exceeds the selected original/rectangle profile");
}
}
Forecast RunArchive::PreflightWithWall(const cases::vehicle_wall::VehicleWallSetup& setup,
    const physical_frames::Mapping& mapping,const records::Context& context,records::source::BundleRequest request,
    Profile profile,Limits limits) {
    CheckSetup(setup,mapping,context,request,profile);
    auto forecast=PreflightCore(mapping.source_mapping(),context,std::move(request),profile,limits,true);
    forecast.shared_wall_setup_upper_bound=setup.forecast().shared_source_upper_bound+
        setup.forecast().retained_setup_bytes;
    return forecast;
}
RunArchive RunArchive::PrepareWithWall(const std::filesystem::path& root,const cases::vehicle_wall::VehicleWallSetup& setup,
    const physical_frames::Mapping& mapping,const records::Context& context,records::source::BundleRequest request,
    Profile profile,Limits limits) {
    CheckSetup(setup,mapping,context,request,profile);
    auto archive=PrepareCore(root,mapping.source_mapping(),context,std::move(request),profile,limits,true);
    Require(std::filesystem::create_directory(root/"wall"),"Physical wall output directory already exists");
    cases::vehicle_wall::WriteSetupArtifacts(root/"wall",setup);
    WallReceipt receipt;
    receipt.source_instance_id=context.identity().source_instance;
    receipt.source_mapping_sha256=context.identity().source_mapping_sha256;
    receipt.wall_binding_id=setup.settings().wall_binding_id;
    for(std::size_t i=0;i<WallFiles.size();++i) {
        const auto bytes=ReadBounded(arrays::CheckedPath(root,WallFiles[i],true),WallFileCap);
        receipt.files[i]={WallFiles[i],Sha256(bytes),bytes.size()};
    }
    ReadWallArtifacts(root,receipt,mapping.source_mapping().source().data(),context);
    archive.data_->manifest.wall=std::move(receipt);
    archive.data_->wall=setup.identity();
    archive.data_->forecast.shared_wall_setup_upper_bound=setup.forecast().shared_source_upper_bound+
        setup.forecast().retained_setup_bytes;
    return archive;
}
} // namespace crash::output::physical_run
