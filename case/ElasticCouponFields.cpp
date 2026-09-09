#include "ElasticCouponFields.h"
#include "ShellPatchFields.h"
#include "output/ArtifactIO.h"
#include <cmath>

namespace crash::case_data {
using namespace crash::output;

Document CouponFrameFields(const ElasticCouponFrame& f) {
    const auto& a=f.element_association;
    using Phase=tl::fea::reissner::ShellBatchPhase;
    Require(f.stamp.owner_id&&a.valid&&a.owner_id==f.stamp.owner_id&&a.configuration_id&&
            a.configuration_id==f.metrics.diagnostics.configuration_id&&a.attempt&&
            ((a.phase==Phase::kAcceptedBase&&a.base_epoch==f.stamp.epoch)||
             (a.phase==Phase::kPreparedCandidate&&f.stamp.epoch>0&&a.base_epoch==f.stamp.epoch-1)),
            "Coupon element fields do not identify the accepted configuration");
    Document doc; doc.SetObject();
    String(doc,"schema","robo_dyna.elastic_coupon_fields.v1");
    Integer(doc,"owner_id",f.stamp.owner_id); Integer(doc,"accepted_epoch",f.stamp.epoch);
    Number(doc,"accepted_time_s",f.stamp.time); Number(doc,"fixed_dt_s",f.stamp.fixed_dt);
    Boolean(doc,"reactions_valid",f.stamp.reactions_valid);
    Integer(doc,"reaction_base_epoch",f.stamp.reaction_base_epoch); Number(doc,"reaction_time_s",f.stamp.reaction_time);
    FiniteArray(doc,"position_xyz_m",f.position.data(),f.position.size());
    FiniteArray(doc,"orientation_wxyz",f.rotation.data(),f.rotation.size());
    FiniteArray(doc,"velocity_xyz_m_per_s",f.velocity.data(),f.velocity.size());
    FiniteArray(doc,"omega_world_xyz_rad_per_s",f.omega.data(),f.omega.size());
    FiniteArray(doc,"reaction_force_xyz_N_at_base",f.reaction_force.data(),f.reaction_force.size());
    FiniteArray(doc,"reaction_couple_world_xyz_Nm_at_base",f.reaction_couple.data(),f.reaction_couple.size());
    const auto& association=f.element_association;
    Integer(doc,"element_evaluation_base_epoch",association.base_epoch);
    Integer(doc,"element_evaluation_attempt",association.attempt);
    Integer(doc,"qualification_id",association.configuration_id);
    String(doc,"element_evaluation_phase",association.phase==tl::fea::reissner::ShellBatchPhase::kAcceptedBase?
           "accepted_base":"prepared_candidate_subsequently_committed");
    String(doc,"element_order","Synthetic parent elements 1,2; four Gauss points in immutable setup order");
    String(doc,"gauss_row_order","eps1.x,eps1.y,eps1.z,eps2.x,eps2.y,eps2.z,k1.x,k1.y,k1.z,k2.x,k2.y,k2.z");
    String(doc,"gauss_basis","Corotated material frame at each Gauss point; transverse rows eps1.z/eps2.z use ANS");
    String(doc,"gauss_strain_units","Rows 0-5 dimensionless; rows 6-11 1/m");
    String(doc,"gauss_resultant_units","Rows 0-5 N/m; rows 6-11 N (bending/twisting couple per width); these are section resultants, not Cauchy stress");
    const std::uint64_t parents[]{1,2};
    AppendShellElements(doc,f.element.data(),f.element.size(),parents); return doc;
}

Document CouponConfiguration(const ElasticCouponCase& run,unsigned frame_every) {
    Require(run.metrics()&&run.modal()&&run.output(),"Coupon output requires an initialized run");
    const auto& m=*run.modal(); Document doc; doc.SetObject();
    String(doc,"schema","robo_dyna.elastic_coupon_configuration.v1");
    String(doc,"scope","Synthetic two-Q4 clamped elastic release; no contact, plasticity or vehicle model");
    String(doc,"units","SI; orientations wxyz; couples and angular velocities world-frame");
    String(doc,"inertia_policy","Physical rho*t^3*A/12 tangential inertia; equal numerical drilling gives total J*I");
    using D=reference::ElasticCouponData;
    Number(doc,"length_m",D::length); Number(doc,"width_m",D::width); Number(doc,"thickness_m",D::thickness);
    Number(doc,"young_modulus_Pa",D::young_modulus); Number(doc,"poisson_ratio",D::poisson_ratio); Number(doc,"density_kg_per_m3",D::density);
    Number(doc,"initial_tip_displacement_m",D::initial_tip_displacement);
    Number(doc,"fixed_dt_s",run.metrics()->stamp.fixed_dt); Number(doc,"half_period_horizon_s",m.horizon);
    Integer(doc,"required_steps",run.metrics()->required_steps); Integer(doc,"frame_every",frame_every);
    Number(doc,"modal_angular_frequency_rad_per_s",m.first_mode_angular_frequency);
    Number(doc,"initial_energy_J",run.metrics()->initial_energy);
    Number(doc,"monitored_norm_limit_per_s2",m.monitored_norm_limit);
    Number(doc,"spectral_step_limit_s",m.spectral_step_limit); Number(doc,"wave_step_limit_s",m.wave_step_limit);
    Number(doc,"rotary_step_limit_s",m.rotary_step_limit); Number(doc,"proposed_step_limit_s",m.proposed_step_limit);
    Number(doc,"reference_symmetry_error",m.reference_symmetry_error);
    Number(doc,"derivative_refinement_error",m.mass_scaled_derivative_refinement_error);
    Number(doc,"TL_derivative_error",m.tl_derivative_relative_error); Number(doc,"TL_directional_error",m.tl_directional_relative_error);
    FiniteArray(doc,"squared_frequency_per_s2",m.squared_frequency.data(),m.squared_frequency.size());
    FiniteArray(doc,"sampled_operator_norm_per_s2",m.sampled_operator_norm.data(),m.sampled_operator_norm.size());
    FiniteArray(doc,"initial_mode_increment_m_rad",m.initial_mode_increment.data(),m.initial_mode_increment.size());
    AppendShellReference(doc,*run.model_data());
    Number(doc,"maximum_displacement_m",ElasticCouponLimits::displacement);
    Number(doc,"maximum_relative_energy_error",ElasticCouponLimits::energy_fraction);
    Number(doc,"maximum_director_departure_rad",ElasticCouponLimits::director_departure);
    Number(doc,"maximum_pair_angle_rad",ElasticCouponLimits::pair_angle);
    Number(doc,"maximum_step_rotation_rad",ElasticCouponLimits::rotation_increment);
    Number(doc,"maximum_membrane_shear_strain",ElasticCouponLimits::strain);
    Number(doc,"maximum_thickness_curvature",ElasticCouponLimits::thickness_curvature);
    Number(doc,"minimum_area_ratio",ElasticCouponLimits::minimum_area_ratio); Number(doc,"maximum_area_ratio",ElasticCouponLimits::maximum_area_ratio);
    String(doc,"admission_scope","Sampled spectral envelope plus candidate geometry, energy and work gates; not a general nonlinear stability proof");
    String(doc,"field_phase","Per-frame element fields at accepted geometry; clamp reactions at preceding base state; intervals give base force work and accepted energy");
    const auto& b=*run.output()->surface().binding();
    Integer(doc,"owner_id",b.identity.owner); Integer(doc,"run_id",b.identity.run); Integer(doc,"topology_id",b.identity.topology);
    String(doc,"source_identity","Synthetic fixture asset/instance 1; not Yaris deck identities");
    AppendSurfaceBinding(doc,b);
    return doc;
}
}  // namespace crash::case_data
