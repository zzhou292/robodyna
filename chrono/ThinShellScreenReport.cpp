#include "ThinShellScreenReportDetail.h"

namespace crash::reference {
namespace {
using namespace screen_report;
Document Parameters(const ElasticCouponParameters& v) {
    auto d=Object(); Number(d,"length_m",v.length); Number(d,"width_m",v.width);
    Number(d,"thickness_m",v.thickness); Number(d,"young_modulus_Pa",v.young_modulus);
    Number(d,"poisson_ratio",v.poisson_ratio); Number(d,"density_kg_per_m3",v.density);
    Number(d,"shear_factor",v.shear_factor); Number(d,"torque_factor",v.torque_factor); return d;
}
Document Mass(const patch_audit::PatchNodalMass& v) {
    auto d=Object(); FiniteArray(d,"mass_kg",v.mass.data(),kCouponNodes);
    FiniteArray(d,"total_isotropic_inertia_kg_m2",v.total_isotropic_inertia.data(),kCouponNodes); return d;
}
Document Inertia(const ShellPatchInertia& v) {
    auto d=Object(); Boolean(d,"prepared",v.prepared);
    FiniteArray(d,"element_area_m2",v.element_area.data(),kCouponElements);
    Objects(d,"original_nodal_contributions",v.original,[](const auto& n) {
        auto item=Object(); Number(item,"area_m2",n.area); Number(item,"mass_kg",n.mass);
        Number(item,"physical_tangential_inertia_kg_m2",n.physical_tangential_inertia);
        Number(item,"artificial_drilling_inertia_kg_m2",n.artificial_drilling_inertia); return item;
    });
    FiniteArray(d,"added_tangential_inertia_kg_m2",v.added_tangential_inertia.data(),kCouponNodes);
    FiniteArray(d,"added_drilling_inertia_kg_m2",v.added_drilling_inertia.data(),kCouponNodes);
    Nested(d,"counterfactual",Mass(v.counterfactual)); return d;
}
Document Measurement(const ThinShellMatrixMeasurement& v) {
    auto d=Object(); Boolean(d,"available",v.available); Integer(d,"status",static_cast<unsigned>(v.status));
    String(d,"diagnostic",v.diagnostic); MatrixRows(d,"stiffness",v.stiffness); return d;
}
Document Step(const ThinShellStepDiagnostic& v) {
    auto d=Object(); Boolean(d,"available",v.available); String(d,"diagnostic",v.diagnostic);
    Number(d,"neutral_operator_norm_per_s2",v.neutral_operator_norm);
    Number(d,"neutral_spectral_limit_s",v.neutral_spectral_limit);
    Number(d,"neutral_spectral_steps_200ms",v.neutral_spectral_steps_200ms);
    Number(d,"membrane_wave_speed_m_per_s",v.membrane_wave_speed); Number(d,"wave_limit_s",v.wave_limit);
    Number(d,"physical_rotary_limit_s",v.physical_rotary_limit);
    Boolean(d,"original_policy_estimate_available",v.original_policy_estimate_available);
    Number(d,"original_policy_limit_s",v.original_policy_limit);
    Number(d,"original_policy_steps_200ms",v.original_policy_steps_200ms); return d;
}
Document Policy(const ThinShellPolicyDiagnostic& v) {
    Require(v.policy==ThinShellInertiaPolicy::PhysicalThickness||v.policy==ThinShellInertiaPolicy::AreaCounterfactual,
            "Thin-shell report has an unknown inertia policy");
    auto d=Object(); String(d,"policy",v.policy==ThinShellInertiaPolicy::PhysicalThickness?
        "original_physical_thickness_and_equal_drilling":"reissner_with_qeph_centered_area_inertia_counterfactual");
    Nested(d,"mass",Mass(v.mass)); FiniteArray(d,"inverse_root_mass",v.inverse_root_mass.data(),kCouponFreeDofs);
    Objects(d,"raw_spectrum_coarse_fine",v.spectrum,Spectrum);
    Boolean(d,"reference_attempted",v.reference_attempted); Boolean(d,"reference_passed",v.reference_passed);
    Integer(d,"reference_status",static_cast<unsigned>(v.reference_status)); String(d,"reference_diagnostic",v.reference_diagnostic);
    Number(d,"reference_symmetry_error",v.reference_symmetry_error);
    Number(d,"derivative_refinement_error",v.derivative_refinement_error); Number(d,"tl_derivative_error",v.tl_derivative_error);
    Number(d,"maximum_frequency_refinement_error",v.maximum_frequency_refinement_error);
    Number(d,"reference_eigen_residual",v.reference_eigen_residual);
    Boolean(d,"directional_attempted",v.directional_attempted);
    Integer(d,"directional_status",static_cast<unsigned>(v.directional_status));
    Number(d,"directional_error",v.directional_error); String(d,"directional_diagnostic",v.directional_diagnostic);
    Nested(d,"coarse_fine_match",Match(v.coarse_fine_match)); Nested(d,"neutral_step_screen",Step(v.step)); return d;
}
Document Fixture(const ThinShellFixtureDiagnostic& v) {
    Require(!v.simulation_ready,"A thin-shell host screen cannot claim simulation readiness");
    auto d=Object(); Integer(d,"fixture_index",v.fixture_index); Number(d,"edge_m",v.edge_m);
    Number(d,"thickness_multiplier",v.thickness_multiplier); Nested(d,"parameters",Parameters(v.parameters));
    Number(d,"translation_difference_m",v.difference_steps.translation_m);
    Number(d,"rotation_difference_rad",v.difference_steps.rotation_rad);
    Boolean(d,"setup_available",v.setup_available); Boolean(d,"screen_passed",v.screen_passed);
    Boolean(d,"simulation_ready",false); String(d,"diagnostic",v.diagnostic);
    Value position(rapidjson::kArrayType),rotation(rapidjson::kArrayType),connectivity(rapidjson::kArrayType);
    for(std::size_t n=0;n<kCouponNodes;++n) {
        const auto& x=v.reference_configuration.position[n]; const double xyz[]{x.x,x.y,x.z};
        const auto& q=v.reference_configuration.rotation[n]; const double wxyz[]{q.w,q.x,q.y,q.z};
        position.PushBack(FiniteArray(d,xyz,3),d.GetAllocator()); rotation.PushBack(FiniteArray(d,wxyz,4),d.GetAllocator());
    }
    for(const auto& nodes:v.connectivity) {
        Value row(rapidjson::kArrayType);
        for(const auto n:nodes) row.PushBack(static_cast<std::uint64_t>(n),d.GetAllocator());
        connectivity.PushBack(row,d.GetAllocator());
    }
    d.AddMember("reference_position_m",position,d.GetAllocator()); d.AddMember("reference_rotation_wxyz",rotation,d.GetAllocator());
    d.AddMember("connectivity_zero_based",connectivity,d.GetAllocator()); Nested(d,"inertia",Inertia(v.inertia));
    Objects(d,"raw_measurements_chrono_coarse_chrono_fine_tl_fine",v.measurement,Measurement);
    Objects(d,"policies",v.policies,Policy); Nested(d,"hybrid_match",Match(v.hybrid_match)); return d;
}
} // namespace
output::Document ThinShellScreenReport(const ThinShellScreenDiagnostic& v,const output::Document& provenance) {
    using namespace screen_report;
    Require(!v.simulation_ready,"A thin-shell host screen cannot claim simulation readiness");
    Require(provenance.IsObject()&&!provenance.ObjectEmpty(),"Thin-shell report requires source/build provenance");
    auto d=Object(); String(d,"schema","robo_dyna.thin_shell_screen.v1");
    String(d,"scope","Six synthetic elastic two-Q4 source-scale reference patches; no dynamics or vehicle admission");
    String(d,"formulation","Actual coherent Chrono Reissner reference and TL prescribed force parity");
    String(d,"counterfactual_scope","Reissner stiffness with centered QEPH area inertia only; not a QEPH element implementation");
    String(d,"donor_reference_commit","a62b27e6baa555d222a580d6218867d0be4d70b5");
    String(d,"donor_inertia_branch","INER_9_12=0; delta_J_ei=m_ei*A_e/12; whole element area before nodal assembly");
    Boolean(d,"screen_passed",v.screen_passed); Boolean(d,"simulation_ready",false);
    Number(d,"source_thickness_m",kThinShellSourceThickness); Number(d,"forecast_horizon_s",kThinShellForecastHorizon);
    Number(d,"derivative_relative_tolerance",patch_audit::DerivativeTolerance);
    Number(d,"tracked_frequency_refinement_tolerance",patch_audit::FrequencyRefinementTolerance);
    Number(d,"cluster_relative_gap",kShellClusterRelativeGap);
    Number(d,"minimum_squared_modal_cosine",kShellModeMinimumSquaredCosine);
    Number(d,"maximum_hybrid_relative_frequency_change",kShellModeMaximumRelativeFrequencyChange);
    Number(d,"minimum_projection_rank_ratio",kShellModeMinimumRankRatio);
    Number(d,"finite_amplitude_fraction",kThinShellShearAmplitudeFraction);
    String(d,"finite_amplitude_scope","TL shear rows 2 and 5 at amplitude and half amplitude; diagnostic only, not a linearity gate");
    String(d,"raw_spectrum_scope","Raw symmetric eigendiagnostics remain visible after audit rejection; only reference_passed identifies an admitted reference spectrum");
    Indices(d,"free_node_order",kCouponFreeNodes,kCouponFreeNodes.size());
    String(d,"free_coordinate_order","Each free node: world x,y,z,spin_x,spin_y,spin_z");
    String(d,"mass_modes_interpretation","Rows follow free coordinate order; columns are mass-normalized eigenmodes; physical modes equal inverse_root_mass times each column");
    String(d,"finite_amplitude_normalization","max(max_node displacement_norm,edge_m*max_node rotation_vector_norm); amplitude=1e-4*min(edge_m,thickness_m), then half");
    String(d,"timestep_scope","Neutral-only optimistic cost estimates; no nonlinear sampled envelope or counterfactual rotary admission");
    Nested(d,"provenance",provenance); Objects(d,"fixtures",v.fixtures,Fixture); return d;
}
} // namespace crash::reference
