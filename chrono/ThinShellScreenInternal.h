#pragma once

#include "ThinShellScreen.h"

namespace crash::reference::thin_shell_detail {
// Private pure host operations are separate so the selection contract can be
// tested with analytic modes without constructing a synthetic dynamics owner.
bool InspectSpectrum(const ThinShellMatrix& stiffness,const ThinShellVector& inverse_root_mass,
                     ThinShellSpectrumDiagnostic& output,std::string& diagnostic);
bool ClassifyModes(ThinShellSpectrumDiagnostic&,const ThinShellVector& inverse_root_mass,std::string& diagnostic);
void DescribeModeEnergies(const ElasticCouponModel&,const ShellPatchInertia&,ThinShellInertiaPolicy,
                         const ThinShellVector& inverse_root_mass,bool finite_amplitude,
                         ThinShellSpectrumDiagnostic&);
void MatchCluster(const ThinShellSpectrumDiagnostic& reference,const ThinShellVector& reference_inverse_root,
                  std::uint32_t reference_cluster,const ThinShellSpectrumDiagnostic& candidate,
                  const ThinShellVector& candidate_inverse_root,const patch_audit::PatchNodalMass& physical_mass,
                  bool require_fd_refinement,ThinShellModeMatchDiagnostic&);
void EstimateSteps(const ElasticCouponParameters&,double edge,ThinShellInertiaPolicy,
                   const ThinShellMatrix& fine,const ThinShellVector& inverse_root_mass,ThinShellStepDiagnostic&);
void CompleteScreen(ThinShellFixtureDiagnostic&);
} // namespace crash::reference::thin_shell_detail
