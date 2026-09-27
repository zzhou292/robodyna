#pragma once
#include "../TiedSearchPostKinChk.h"
#include <string>

namespace crash::cases::vehicle_startup::physical_attachments {
// Existing bounded search producers, including the CUDA startup assessment.
// The returned handle retains assessment, geometry, packing and declaration.
TiedSearchFinalized FinalizeOriginalTiedSearch(
    const modelio::tied_shell::TiedShellDeclaration&, const std::string& member);

// Callers prepare auxiliary constraints and context at their original boundary.
// In particular, wall byte temporaries expire before this phase begins.
TiedSearchPostKinChk PrepareOriginalTiedPost(
    const TiedSearchFinalized&, const modelio::tied_shell::TiedClassificationContext&);
} // namespace crash::cases::vehicle_startup::physical_attachments
