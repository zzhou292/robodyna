#pragma once

#include "ElasticShellEnvelope.h"
#include "chrono/GuidedPlateModal.h"
#include "lib_src/collision/Q4PlanarContact.h"

namespace crash::case_data {
// Derived from a matching pair of contributor diagnostics, never a receipt
// or authority to commit. The coordinator must bind the prepared owner token.
struct GuidedPlateWorkReport {
    double kinetic_energy = 0, total_energy = 0, relative_energy_error = 0;
    double combined_kinetic_residual = 0, kinetic_arithmetic_budget = 0;
    double shell_coordinate_work_budget = 0;
    double contact_defect_lower_limit = 0, contact_defect_upper_limit = 0;
};

// Reuses the elastic geometric limits; energy includes contact potential and
// kinetic work includes both actual BASE force contributions. Continuum
// integration uncertainty cannot excuse a failed discrete kinetic identity.
// Output is staged until every gate passes; a failure supplies a diagnostic.
bool CheckGuidedPlateEnvelope(const tl::fea::reissner::ShellBatchDiagnostics&,
                              const tlfea::contact::Q4PlanarContactDiagnostics&,
                              const reference::GuidedPlateModalReport&,
                              double initial_energy,double maximum_displacement,bool candidate,
                              GuidedPlateWorkReport& output,std::string& diagnostic);
}  // namespace crash::case_data
