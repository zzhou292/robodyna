#include "PilotSource.h"
#include "output/ArtifactIO.h"

namespace crash::cases::source_assembly_wall {
modelio::assembly::SourceAssembly ReadPilotSource(const std::string& path,PilotAssembly scope) {
    namespace source=modelio::assembly;
    output::Require(scope==PilotAssembly::SixPart||scope==PilotAssembly::SevenPartSpotweld,"Invalid pilot source selection");
    return source::SourceAssembly::Read(path,scope==PilotAssembly::SixPart?
        source::PinnedYarisSixPartInventory():source::PinnedYarisSevenPartInventory());
}
source_assembly::SourceAssemblyBindings PreparePilotBindings(const modelio::assembly::SourceAssembly& input,PilotAssembly scope) {
    namespace source=modelio::assembly;
    output::Require(scope==PilotAssembly::SixPart||scope==PilotAssembly::SevenPartSpotweld,"Invalid pilot source selection");
    const auto expected=scope==PilotAssembly::SixPart?source::PinnedYarisSixPartInventory():source::PinnedYarisSevenPartInventory();
    output::Require(input.data().identity.sha256==expected.sha256&&input.data().identity.bytes==expected.bytes,
                    "Pilot source differs from its explicit pinned selection");
    source_assembly::SourceAssemblyBindingOptions options{
        scope==PilotAssembly::SixPart?0x5941524953ULL:0x594152495337ULL,source::MaterialRatePolicy::OpenRadiossDirectImportDefault};
    if(scope==PilotAssembly::SevenPartSpotweld)options.spotweld=source::SpotweldDeclaration{
        source::SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,383348001108};
    return source_assembly::SourceAssemblyBindings::Prepare(input,options);
}
} // namespace crash::cases::source_assembly_wall
