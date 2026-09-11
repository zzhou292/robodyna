#include "Support.h"

namespace crash::cases::vehicle_startup::physical_attachments::test {
const TiedSearchPostKinChk& Post() {
    static const auto value = [] {
        const auto& source = physical_model::test::Source().source();
        const auto& original = modelio::physical_scope::test::Inputs();
        const auto geometry = tied::TiedShellSearchGeometry::Prepare(
            tied::TiedShellPacking::Prepare(source.tied_source()), original.member);
        const auto finalized = TiedSearchFinalized::Prepare(TiedSearchAssessment::Prepare(geometry));
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
        return TiedSearchPostKinChk::Prepare(TiedSearchClassification::Prepare(finalized, context));
    }();
    return value;
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
