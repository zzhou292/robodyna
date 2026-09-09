#pragma once

#include "GuidedPlateCase.h"
#include "GuidedPlateContactProtocol.h"
#include "GuidedPlateExperimentProtocol.h"
#include "output/ArtifactIO.h"

namespace crash::case_data {
// Accepted-result and immutable-configuration serialization. No device reads,
// mechanics or phase inference from visual mesh; caller supplies captured data.
output::Document GuidedPlateConfiguration(const GuidedPlateCase&,unsigned frame_every,
                                          const std::string& canonical_manifest_sha256);
output::Document GuidedPlateFrameFields(const GuidedPlateFrame&,const reference::GuidedPlateData&);
} // namespace crash::case_data
