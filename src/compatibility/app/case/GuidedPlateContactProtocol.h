#pragma once
#include "GuidedPlateContactIdentity.h"
#include "output/ContactIntegrationMetadata.h"
#include <string_view>

namespace crash::case_data {
// Application enum/protocol adapter only. Archive spelling and legacy rules
// belong to the solver-independent output protocol; no mechanics are selected.
inline const char* GuidedContactBackendName(tlfea::contact::Q4PlanarIntegrationBackend backend) {
    output::Require(ValidGuidedContactBackend(backend),"Unknown guided contact integration backend");
    return backend==tlfea::contact::Q4PlanarIntegrationBackend::ScalarDyadicSquares ?
        output::contact_metadata::Scalar : output::contact_metadata::Rectangular;
}
inline bool ParseGuidedContactBackend(std::string_view name,tlfea::contact::Q4PlanarIntegrationBackend& out) noexcept {
    using Backend=tlfea::contact::Q4PlanarIntegrationBackend;
    if(name==output::contact_metadata::Scalar){out=Backend::ScalarDyadicSquares;return true;}
    if(name==output::contact_metadata::Rectangular){out=Backend::RectangularDyadic;return true;}
    return false;
}
} // namespace crash::case_data
