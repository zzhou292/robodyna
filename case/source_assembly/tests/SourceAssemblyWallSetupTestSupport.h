#pragma once
#include "SourceAssemblyBindingTestSupport.h"
#include "case/source_assembly/SourceAssemblyWallSetup.h"
#include "case/CanonicalWallArtifacts.h"
#include <cstdlib>
#include <sstream>

namespace crash::cases::source_assembly::test {
inline const SourceAssemblyBindings& WallAssembly() {
    static const auto value=SourceAssemblyBindings::Prepare(Load(),Options());return value;
}
struct WallInput {
    std::string bytes;
    case_data::CanonicalWall canonical;
    WallInput() {
        const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_WALL");
        if(!path)throw std::runtime_error("Frozen assembly wall manifest is required");
        bytes=case_data::ReadPinnedWallManifest(path);std::istringstream input(bytes);
        if(canonical.Load(input).status!=case_data::WallStatus::Ok)throw std::runtime_error("Frozen canonical wall failed to load");
    }
};
inline SourceAssemblyWallSettings WallSettings() {
    SourceAssemblyWallSettings settings;settings.initial_velocity={8,0,0};
    settings.leading_gap=5e-6; // Explicit smoke placement; setup never chooses a run horizon.
    settings.configuration_id=8101;settings.qualification_id=8102;settings.wall_binding_id=8103;
    settings.boundary=SourceAssemblyWallBoundary::ReleasedExternalConnections;return settings;
}
inline fe::NodalStamp DeclaredStamp(const SourceAssemblyBindings& b) {
    fe::NodalStamp stamp;stamp.owner_id=8001;stamp.node_count=b.shells().node_count();
    stamp.fixed_dt=1./67108864;stamp.has_rotations=true;
    stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    stamp.rigid_groups={b.source_instance_id(),b.rigid_groups()->group_count(),b.rigid_groups()->member_count()};return stamp;
}
inline void Encloses(tlfea::contact::Q4CertifiedIntegral value,long double truth) {
    EXPECT_LE(static_cast<long double>(value.lower),truth);EXPECT_GE(static_cast<long double>(value.upper),truth);
}
static_assert(std::is_nothrow_copy_constructible_v<SourceAssemblyWallSetup>);
static_assert(std::is_nothrow_move_constructible_v<SourceAssemblyWallSetup>);
static_assert(!std::is_copy_assignable_v<SourceAssemblyWallSetup> && !std::is_move_assignable_v<SourceAssemblyWallSetup>);
} // namespace crash::cases::source_assembly::test
