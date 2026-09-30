#pragma once
#include "../VehiclePhysicalAttachments.h"
#include "../OriginalTiedPost.h"
namespace crash::cases::vehicle_startup::physical_attachments::test {
inline TiedSearchPostKinChk PreparePost(const modelio::physical_scope::PhysicalScope& source,
                                      const std::string& member) {
        const auto finalized = FinalizeOriginalTiedSearch(source.tied_source(), member);
        const auto read = [](const char* name, std::size_t cap) {
            const auto* path = std::getenv(name);
            output::Require(path && *path, "Explicit auxiliary/wall source fixture required");
            return output::ReadBounded(path, cap);
        };
        const auto auxiliary = tied::TiedAuxiliaryConstraints::Prepare(source.tied_source(),
            read("ROBO_TIED_AUX_MEMBER",44991), tied::OriginalWallPolicy::ReplaceWithMeshWall);
        const auto context = tied::TiedClassificationContext::Prepare(auxiliary,
            source.point_mass_source().rigid_source(), read("ROBO_TIED_WALL_MEMBER",10604),
            tied::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall);
        return PrepareOriginalTiedPost(finalized, context);
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
