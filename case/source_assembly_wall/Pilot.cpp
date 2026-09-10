#include "Pilot.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <sstream>

namespace crash::cases::source_assembly_wall {
source_assembly_dynamics::Config PilotConfig(unsigned refinement) {
    output::Require(refinement==1||refinement==2||refinement==4,"Pilot refinement must be 1, 2 or 4");
    source_assembly_dynamics::Config config;config.fixed_dt=1./67108864/refinement;
    auto& d=config.deformation;
    d.maximum_displacement=.02;d.maximum_rotation=1;d.maximum_rotation_increment=1;
    d.maximum_strain=.2;d.maximum_thickness_curvature=.2;
    d.minimum_area_ratio=.5;d.maximum_area_ratio=1.5;
    d.minimum_thickness_ratio=.5;d.maximum_thickness_ratio=1.5;d.maximum_native_dt_fraction=.5;
    return config;
}
void InitializePilot(source_assembly_dynamics::SourceAssemblyWallCase& run,
    const std::string& inventory,const std::string& wall_path,
    const output::assembly::WallArchiveRequest& request,unsigned refinement,
    source_assembly_dynamics::StepTimingOptions timing) {
    namespace source=modelio::assembly;
    const auto config=PilotConfig(refinement);
    const auto input=source::SourceAssembly::Read(inventory,source::PinnedYarisSixPartInventory());
    const auto bytes=case_data::ReadPinnedWallManifest(wall_path);
    // Reject impossible archive requests before CUDA startup and any directory.
    output::assembly::PlanWallArchive(request,input.data().identity.bytes,bytes.size());
    std::istringstream stream(bytes);case_data::CanonicalWall canonical;
    output::Require(canonical.Load(stream).status==case_data::WallStatus::Ok,"Cannot load original finite wall");
    const auto bindings=source_assembly::SourceAssemblyBindings::Prepare(input,
        {0x5941524953,source::MaterialRatePolicy::OpenRadiossDirectImportDefault});
    source_assembly::SourceAssemblyWallSettings settings;
    settings.initial_velocity={8,0,0};settings.leading_gap=5e-6;
    settings.configuration_id=0x534157434631ULL;settings.qualification_id=0x534157434751ULL;
    settings.wall_binding_id=0x53415757414cULL;
    settings.boundary=source_assembly::SourceAssemblyWallBoundary::ReleasedExternalConnections;
    source_assembly::SourceAssemblyWallSetup setup;
    const auto prepared=setup.Initialize(bindings,canonical,bytes,settings);output::Require(bool(prepared),prepared.message);
    const auto initialized=run.Initialize(bindings,setup,config,timing);
    if(!initialized)throw std::runtime_error(FailureText(initialized));
}
}
