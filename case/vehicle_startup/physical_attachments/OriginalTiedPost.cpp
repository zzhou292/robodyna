#include "OriginalTiedPost.h"

namespace crash::cases::vehicle_startup::physical_attachments {
TiedSearchFinalized FinalizeOriginalTiedSearch(
    const modelio::tied_shell::TiedShellDeclaration& declaration, const std::string& member) {
    const auto geometry=tied::TiedShellSearchGeometry::Prepare(
        tied::TiedShellPacking::Prepare(declaration),member);
    return TiedSearchFinalized::Prepare(TiedSearchAssessment::Prepare(geometry));
}
TiedSearchPostKinChk PrepareOriginalTiedPost(
    const TiedSearchFinalized& finalized, const modelio::tied_shell::TiedClassificationContext& context) {
    return TiedSearchPostKinChk::Prepare(TiedSearchClassification::Prepare(finalized,context));
}
} // namespace crash::cases::vehicle_startup::physical_attachments
