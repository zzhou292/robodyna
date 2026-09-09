#include "GuidedPlateFields.h"
#include "CanonicalWallArtifacts.h"
#include "ShellPatchFields.h"
#include "GuidedPlateContactIdentity.h"
#include "output/ContactIntegrationMetadata.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <cmath>

namespace crash::case_data {
using namespace crash::output;
namespace ct=tlfea::contact;
namespace {
Value Vector(Document& doc,ct::Vec3 v) { const double a[]{v.x,v.y,v.z}; return FiniteArray(doc,a,3); }
Value Certificate(Document& doc,const ct::Q4CertifiedIntegral& c) {
    ct::Q4CertifiedIntegral checked;
    Require(std::isfinite(c.error)&&c.error>=0&&ct::q4_bounds::Certify(c.value,{c.lower,c.upper},&checked)&&
            checked.error<=c.error,"Invalid contact certificate output");
    const double a[]{c.value,c.lower,c.upper,c.error}; return FiniteArray(doc,a,4);
}
Value Interval(Document& doc,ct::Q4IntegralInterval c) {
    Require(c.lower>=0 && c.lower<=c.upper,"Invalid contact interval output");
    const double a[]{c.lower,c.upper}; return FiniteArray(doc,a,2);
}
Value ParentBinding(Document& doc,const ct::SurfaceQ4& p) {
    Value item(rapidjson::kObjectType),nodes(rapidjson::kArrayType);
    item.AddMember("parent_element",Value().SetUint64(p.parent_element_id),doc.GetAllocator());
    item.AddMember("parent_face",p.parent_face_id,doc.GetAllocator());
    item.AddMember("feature_id",Value().SetUint64(p.feature_id),doc.GetAllocator());
    for(auto n:p.nodes)nodes.PushBack(n,doc.GetAllocator());
    item.AddMember("connectivity_zero_based",nodes,doc.GetAllocator()); return item;
}
void CheckFrame(const GuidedPlateFrame& f,const reference::GuidedPlateData& model) {
    const auto& s=f.element_association; const auto& c=f.contact_association;
    const bool candidate=s.phase==tl::fea::reissner::ShellBatchPhase::kPreparedCandidate;
    const auto base=candidate&&f.stamp.epoch?f.stamp.epoch-1:f.stamp.epoch;
    Require(f.stamp.owner_id && f.stamp.node_count==reference::kCouponNodes && f.stamp.has_rotations &&
        std::isfinite(f.stamp.fixed_dt)&&f.stamp.fixed_dt>0&&std::isfinite(f.stamp.time)&&f.stamp.time>=0&&s.valid&&c.valid&&s.attempt&&
        s.owner_id==f.stamp.owner_id && c.owner_id==s.owner_id && s.attempt==c.attempt &&
        GuidedExperimentIdentity(model.experiment,model.qualification_id)&&s.configuration_id==model.qualification_id && c.configuration_id==s.configuration_id &&
        c.wall_binding_id==kGuidedPlateWallBinding && s.base_epoch==base && c.base_epoch==base &&
        ((candidate&&f.stamp.epoch&&c.phase==ct::Q4PlanarContactPhase::PreparedCandidate)||
         (!candidate&&s.phase==tl::fea::reissner::ShellBatchPhase::kAcceptedBase&&c.phase==ct::Q4PlanarContactPhase::AcceptedBase)),
        "Guided fields do not identify matching accepted endpoint evaluations");
    Require(f.metrics.shell.configuration_id==model.qualification_id&&f.metrics.contact.configuration_id==model.qualification_id&&
        ValidGuidedContactPartition(c,c.integration_backend)&&
        ValidGuidedContactPartition(f.metrics.contact,c.integration_backend),"Guided field contact backend or depths mismatch");
    Require(f.metrics.stamp.owner_id==f.stamp.owner_id && f.metrics.stamp.epoch==f.stamp.epoch &&
        f.metrics.stamp.node_count==f.stamp.node_count&&f.metrics.stamp.has_rotations==f.stamp.has_rotations&&
        Bits(f.metrics.stamp.time)==Bits(f.stamp.time)&&Bits(f.metrics.stamp.fixed_dt)==Bits(f.stamp.fixed_dt)&&
        f.metrics.stamp.reactions_valid==f.stamp.reactions_valid&&f.metrics.stamp.reaction_base_epoch==f.stamp.reaction_base_epoch&&
        Bits(f.metrics.stamp.reaction_time)==Bits(f.stamp.reaction_time)&&
        ((!f.stamp.epoch&&!f.stamp.reactions_valid&&!f.stamp.reaction_base_epoch&&f.stamp.reaction_time==0)||
         (f.stamp.epoch&&f.stamp.reactions_valid&&f.stamp.reaction_base_epoch==f.stamp.epoch-1&&
          f.stamp.reaction_time+f.stamp.fixed_dt==f.stamp.time)),"Guided fields have invalid reaction or metric phase");
}
void AppendContact(Document& doc,const GuidedPlateFrame& f,const reference::GuidedPlateData& model) {
    const auto& d=f.contact_association;
    Require(d.parent_count==model.parents.size()&&d.covered_count==model.parents.size()&&
            ValidGuidedContactPartition(d,f.parent.data(),f.parent.size(),d.integration_backend),
            "Missing guided covered parents or inconsistent contact partition");
    for(const auto error:{d.force_error,d.wall_moment_error})
        Require(error.x>=0&&error.y>=0&&error.z>=0,"Negative guided contact force/moment uncertainty");
    Value parents(rapidjson::kArrayType);
    for(unsigned p=0;p<model.parents.size();++p) {
        const auto& source=model.parents[p]; const auto& result=f.parent[p]; const auto& r=result.integration;
        Require(result.covered&&r.valid&&r.parent_element_id==source.parent_element_id&&r.feature_id==source.feature_id&&
                r.base_epoch==d.base_epoch&&r.attempt==d.attempt&&ValidGuidedContactPartition(result,d.integration_backend),
                "Guided contact parent association mismatch");
        Value item=ParentBinding(doc,source),force(rapidjson::kArrayType),couple(rapidjson::kArrayType),cert(rapidjson::kArrayType);
        item.AddMember("covered",result.covered,doc.GetAllocator()); item.AddMember("valid",r.valid,doc.GetAllocator());
        item.AddMember("base_epoch",Value().SetUint64(r.base_epoch),doc.GetAllocator());
        item.AddMember("attempt",Value().SetUint64(r.attempt),doc.GetAllocator());
        for(unsigned n=0;n<4;++n) {
            Require(r.nodal.nodes[n]==source.nodes[n],"Guided parent node binding mismatch");
            force.PushBack(Vector(doc,r.nodal.forces[n]),doc.GetAllocator());
            couple.PushBack(Vector(doc,r.nodal.couples[n]),doc.GetAllocator());
            cert.PushBack(Certificate(doc,r.force[n]),doc.GetAllocator());
        }
        item.AddMember("force_world_N",force,doc.GetAllocator()); item.AddMember("couple_world_Nm",couple,doc.GetAllocator());
        item.AddMember("nodal_force_magnitude_N",cert,doc.GetAllocator());
        item.AddMember("resultant_magnitude_N",Certificate(doc,r.resultant),doc.GetAllocator());
        item.AddMember("potential_J",Certificate(doc,r.potential),doc.GetAllocator());
        item.AddMember("active_area_m2",Interval(doc,r.active_area),doc.GetAllocator());
        item.AddMember("leaf_count",r.leaf_count,doc.GetAllocator()); item.AddMember("visited",r.visited,doc.GetAllocator());
        item.AddMember("deepest_leaf",r.deepest_leaf,doc.GetAllocator());
        item.AddMember("deepest_u",result.deepest_u,doc.GetAllocator()); item.AddMember("deepest_v",result.deepest_v,doc.GetAllocator());
        item.AddMember(rapidjson::StringRef(contact_metadata::BackendField),
                       Value(GuidedContactBackendName(result.integration_backend),doc.GetAllocator()),doc.GetAllocator());
        parents.PushBack(item,doc.GetAllocator());
    }
    doc.AddMember("contact_parents",parents,doc.GetAllocator());
    Value result(rapidjson::kObjectType);
    result.AddMember(rapidjson::StringRef(contact_metadata::BackendField),
                     Value(GuidedContactBackendName(d.integration_backend),doc.GetAllocator()),doc.GetAllocator());
    result.AddMember("deepest_leaf",d.deepest_leaf,doc.GetAllocator());
    result.AddMember("deepest_u",d.deepest_u,doc.GetAllocator()); result.AddMember("deepest_v",d.deepest_v,doc.GetAllocator());
    result.AddMember("wall_reaction_N",Vector(doc,d.wall_reaction),doc.GetAllocator());
    result.AddMember("wall_moment_Nm",Vector(doc,d.wall_moment),doc.GetAllocator());
    result.AddMember("force_error_N",Vector(doc,d.force_error),doc.GetAllocator());
    result.AddMember("wall_moment_error_Nm",Vector(doc,d.wall_moment_error),doc.GetAllocator());
    result.AddMember("potential_J",Certificate(doc,d.potential),doc.GetAllocator());
    result.AddMember("active_area_m2",Interval(doc,d.active_area),doc.GetAllocator());
    Require(std::isfinite(d.maximum_penetration)&&d.maximum_penetration>=0,"Invalid contact depth output");
    result.AddMember("maximum_penetration_m",d.maximum_penetration,doc.GetAllocator());
    doc.AddMember("contact_result",result,doc.GetAllocator());
}
void AppendContactReference(Document& doc,const GuidedPlateCase& run) {
    const auto& guided=*run.guided_data(); const auto geometry=run.contact_reference();
    Require(geometry.parents&&geometry.parent_count==guided.parents.size()&&geometry.global_node_count==reference::kCouponNodes,
            "Guided output requires the retained immutable C3 reference");
    Value parents(rapidjson::kArrayType),areas(rapidjson::kArrayType);
    for(unsigned p=0;p<guided.parents.size();++p) {
        parents.PushBack(ParentBinding(doc,guided.parents[p]),doc.GetAllocator());
        const auto& r=geometry.parents[p]; Value area(rapidjson::kObjectType);
        area.AddMember("projected_area_m2",r.projected_area,doc.GetAllocator());
        area.AddMember("exact_coordinate_area_m2",Interval(doc,r.area_enclosure),doc.GetAllocator());
        area.AddMember("covered",r.covered,doc.GetAllocator()); areas.PushBack(area,doc.GetAllocator());
    }
    doc.AddMember("contact_parent_binding",parents,doc.GetAllocator()); doc.AddMember("contact_reference_area",areas,doc.GetAllocator());
}
void AppendPenaltyAudit(Document& doc,const GuidedPlateCase& run) {
    const auto* report=run.penalty_audit();
    Require(report&&report->experiment==run.experiment()&&report->qualification_id==run.guided_data()->qualification_id&&
        report->initial_energy==run.modal()->initial_elastic_energy&&
        report->enforced==(run.experiment()==reference::GuidedPlateExperiment::PenaltyMarginV1)&&
        (!report->enforced||report->target_sufficient),"Missing or mismatched startup penalty screen");
    Document child;child.SetObject();
    String(child,"scope","Prescribed initial-mode energy/force screen; not a global penetration bound or completed trajectory");
    String(child,guided_experiment_metadata::Field,GuidedExperimentName(report->experiment));
    Integer(child,"qualification_id",report->qualification_id);Boolean(child,"enforced",report->enforced);
    Boolean(child,"target_sufficient",report->target_sufficient);Boolean(child,"global_penetration_bound",false);
    Number(child,"initial_energy_J",report->initial_energy);Number(child,"admitted_energy_upper_J",report->admitted_energy_upper);
    Value samples(rapidjson::kArrayType);
    for(const auto& sample:report->sample) {
        Document item;item.SetObject();Number(item,"requested_penetration_m",sample.requested_penetration);
        Number(item,"actual_penetration_m",sample.actual_penetration);Number(item,"modal_scale",sample.modal_scale);
        item.AddMember("maximum_depth_m",Interval(item,sample.maximum_depth),item.GetAllocator());
        Integer(item,"inward_scale_adjustments",sample.inward_scale_adjustments);
        Number(item,"shell_energy_tl_J",sample.shell_energy_tl);Number(item,"shell_energy_chrono_J",sample.shell_energy_chrono);
        Number(item,"shell_energy_allowance_J",sample.shell_energy_allowance);
        item.AddMember("contact_potential_J",Certificate(item,sample.contact_potential),item.GetAllocator());
        item.AddMember("contact_resultant_N",Certificate(item,sample.contact_resultant),item.GetAllocator());
        Number(item,"strip_potential_J",sample.strip_potential);Number(item,"strip_resultant_N",sample.strip_resultant);
        Number(item,"strip_energy_allowance_J",sample.strip_energy_allowance);Number(item,"strip_force_allowance_N",sample.strip_force_allowance);
        Number(item,"total_potential_lower_J",sample.total_potential_lower);Value derivatives(rapidjson::kArrayType);
        for(const auto& derivative:sample.derivative) {
            Document d;d.SetObject();Number(d,"step_m",derivative.step_m);Number(d,"derivative_N",derivative.derivative_N);
            Number(d,"restoring_force_N",derivative.restoring_force_N);Number(d,"error_N",derivative.error_N);
            Number(d,"contact_uncertainty_N",derivative.contact_uncertainty_N);Number(d,"allowance_N",derivative.allowance_N);
            Boolean(d,"backward",derivative.backward);Value copy;copy.CopyFrom(d,item.GetAllocator());derivatives.PushBack(copy,item.GetAllocator());
        }
        item.AddMember("derivatives",derivatives,item.GetAllocator());Value copy;copy.CopyFrom(item,child.GetAllocator());samples.PushBack(copy,child.GetAllocator());
    }
    child.AddMember("samples",samples,child.GetAllocator());Value copy;copy.CopyFrom(child,doc.GetAllocator());
    doc.AddMember("startup_penalty_screen",copy,doc.GetAllocator());
}
} // namespace

Document GuidedPlateFrameFields(const GuidedPlateFrame& f,const reference::GuidedPlateData& model) {
    CheckFrame(f,model); Document doc; doc.SetObject(); String(doc,"schema","robo_dyna.guided_plate_fields.v1");
    String(doc,guided_experiment_metadata::Field,GuidedExperimentName(model.experiment));
    Integer(doc,"owner_id",f.stamp.owner_id); Integer(doc,"accepted_epoch",f.stamp.epoch);
    Number(doc,"accepted_time_s",f.stamp.time); Number(doc,"fixed_dt_s",f.stamp.fixed_dt);
    Boolean(doc,"reactions_valid",f.stamp.reactions_valid); Integer(doc,"reaction_base_epoch",f.stamp.reaction_base_epoch);
    Number(doc,"reaction_time_s",f.stamp.reaction_time);
    FiniteArray(doc,"position_xyz_m",f.position.data(),f.position.size()); FiniteArray(doc,"orientation_wxyz",f.rotation.data(),f.rotation.size());
    FiniteArray(doc,"velocity_xyz_m_per_s",f.velocity.data(),f.velocity.size()); FiniteArray(doc,"omega_world_xyz_rad_per_s",f.omega.data(),f.omega.size());
    FiniteArray(doc,"reaction_force_xyz_N_at_base",f.reaction_force.data(),f.reaction_force.size());
    FiniteArray(doc,"reaction_couple_world_xyz_Nm_at_base",f.reaction_couple.data(),f.reaction_couple.size());
    const auto& s=f.element_association; const auto& c=f.contact_association;
    Integer(doc,"qualification_id",s.configuration_id); Integer(doc,"wall_binding_id",c.wall_binding_id);
    String(doc,contact_metadata::BackendField,GuidedContactBackendName(c.integration_backend));
    Integer(doc,"element_evaluation_base_epoch",s.base_epoch); Integer(doc,"element_evaluation_attempt",s.attempt);
    Integer(doc,"contact_evaluation_base_epoch",c.base_epoch); Integer(doc,"contact_evaluation_attempt",c.attempt);
    const char* phase=s.phase==tl::fea::reissner::ShellBatchPhase::kAcceptedBase?"accepted_base":"prepared_candidate_subsequently_committed";
    String(doc,"element_evaluation_phase",phase); String(doc,"contact_evaluation_phase",phase);
    String(doc,"element_order","Immutable contact_parent_binding order; four Gauss points in immutable shell setup order");
    String(doc,"gauss_row_order","eps1.x,eps1.y,eps1.z,eps2.x,eps2.y,eps2.z,k1.x,k1.y,k1.z,k2.x,k2.y,k2.z");
    String(doc,"gauss_basis","Corotated material frame; transverse strain rows use ANS");
    String(doc,"gauss_strain_units","Rows 0-5 dimensionless; rows 6-11 1/m");
    String(doc,"gauss_resultant_units","Rows 0-5 N/m; rows 6-11 N; section resultants, not Cauchy stress");
    String(doc,"contact_certificate_columns","value,lower,upper,error; active area lower,upper");
    const std::uint64_t ids[]{model.parents[0].parent_element_id,model.parents[1].parent_element_id};
    AppendShellElements(doc,f.element.data(),f.element.size(),ids); AppendContact(doc,f,model); return doc;
}

Document GuidedPlateConfiguration(const GuidedPlateCase& run,unsigned frame_every,const std::string& canonical_sha) {
    Require(run.metrics()&&run.modal()&&run.model_data()&&run.guided_data()&&run.output()&&run.output()->surface().binding()&&
            frame_every&&canonical_sha==kCanonicalWallManifestSha256,"Guided configuration requires bound initialized inputs");
    const auto& m=*run.modal(); const auto& g=*run.guided_data(); const auto& b=*run.output()->surface().binding();
    Document doc; doc.SetObject(); String(doc,"schema","robo_dyna.guided_plate_configuration.v1");
    String(doc,"scope","Synthetic guided two-Q4 elastic plate against the original canonical Yaris mesh wall");
    String(doc,"units","SI; orientations wxyz; angular velocities and couples world-frame");
    String(doc,"inertia_policy","Physical tangential rho*t^3*A/12; equal numerical drilling gives total J*I");
    String(doc,"canonical_wall_manifest_sha256",canonical_sha);
    Integer(doc,"owner_id",b.identity.owner); Integer(doc,"run_id",b.identity.run); Integer(doc,"topology_id",b.identity.topology);
    Require(GuidedExperimentIdentity(g.experiment,g.qualification_id)&&run.experiment()==g.experiment&&
        run.metrics()->shell.configuration_id==g.qualification_id&&run.metrics()->contact.configuration_id==g.qualification_id,
        "Guided configuration experiment identity mismatch");
    Integer(doc,"qualification_id",g.qualification_id); Integer(doc,"wall_binding_id",g.wall_binding_id);
    String(doc,guided_experiment_metadata::Field,GuidedExperimentName(g.experiment));
    Require(ValidGuidedContactPartition(run.metrics()->contact,run.integration_backend()),"Guided configuration backend mismatch");
    String(doc,contact_metadata::BackendField,GuidedContactBackendName(run.integration_backend()));
    String(doc,"contact_depth_convention","Independent U/V resolution; deepest_leaf=max(U,V); scalar squares have U=V");
    Number(doc,"fixed_dt_s",run.metrics()->stamp.fixed_dt); Number(doc,"requested_horizon_s",m.horizon);
    Integer(doc,"required_steps",run.metrics()->required_steps); Integer(doc,"frame_every",frame_every);
    Require(m.step_count&&run.metrics()->required_steps%m.step_count==0&&run.diagnostic_stride(),"Invalid guided accepted schedule metadata");
    Integer(doc,"refinement",run.metrics()->required_steps/m.step_count);
    Integer(doc,"diagnostic_every_accepted_steps",run.diagnostic_stride());
    Number(doc,"initial_energy_J",run.metrics()->initial_energy); Number(doc,"initial_tip_displacement_m",g.initial_tip_displacement);
    Number(doc,"initial_gap_m",g.initial_gap); Number(doc,"penalty_per_area_N_per_m3",g.stiffness_per_area);
    Number(doc,"maximum_penetration_m",g.maximum_penetration); Number(doc,"exposed_clearance_m",g.exposed_clearance);
    Number(doc,"penalty_target_penetration_m",g.target_penetration);
    Number(doc,"friction",0); Number(doc,"damping",0); Number(doc,"thickness_offset_m",0);
    Number(doc,"contact_force_error_budget_N",g.integration.force_error); Number(doc,"contact_potential_error_budget_J",g.integration.energy_error);
    Integer(doc,"contact_max_leaves",g.integration.max_leaves); Integer(doc,"contact_max_depth",g.integration.max_depth);
    Integer(doc,"contact_max_visits",g.integration.max_visited);
    using D=reference::ElasticCouponData;
    Number(doc,"length_m",D::length); Number(doc,"width_m",D::width); Number(doc,"thickness_m",D::thickness);
    Number(doc,"young_modulus_Pa",D::young_modulus); Number(doc,"poisson_ratio",D::poisson_ratio); Number(doc,"density_kg_per_m3",D::density);
    AppendShellReference(doc,*run.model_data()); AppendSurfaceBinding(doc,b); AppendContactReference(doc,run);
    Value masks(rapidjson::kArrayType),fixed(rapidjson::kArrayType),offsets(rapidjson::kArrayType);
    for(auto v:g.translation_fixed_bits)masks.PushBack(v,doc.GetAllocator()); for(auto v:g.rotation_fixed)fixed.PushBack(v,doc.GetAllocator());
    for(const auto& ref:run.model_data()->reference) {
        Value element(rapidjson::kArrayType);
        for(const auto& q:ref.node_frame_offset) {const double values[]{q.w,q.x,q.y,q.z}; element.PushBack(FiniteArray(doc,values,4),doc.GetAllocator());}
        offsets.PushBack(element,doc.GetAllocator());
    }
    doc.AddMember("translation_fixed_bits",masks,doc.GetAllocator()); doc.AddMember("rotation_fixed",fixed,doc.GetAllocator());
    doc.AddMember("element_node_frame_offset_wxyz",offsets,doc.GetAllocator());
    FiniteArray(doc,"squared_frequency_per_s2",m.squared_frequency.data(),m.squared_frequency.size());
    FiniteArray(doc,"initial_mode_increment_m_rad",m.initial_mode_increment.data(),m.initial_mode_increment.size());
    FiniteArray(doc,"sampled_amplitude",m.sampled_amplitude.data(),m.sampled_amplitude.size());
    FiniteArray(doc,"sampled_structural_operator_norm_per_s2",m.sampled_structural_operator_norm.data(),m.sampled_structural_operator_norm.size());
    Number(doc,"modal_angular_frequency_rad_per_s",m.first_mode_angular_frequency);
    Number(doc,"monitored_structural_norm_limit_per_s2",m.monitored_structural_norm_limit);
    Number(doc,"contact_rate_bound_per_s2",m.contact_rate_bound); Number(doc,"combined_rate_envelope_per_s2",m.combined_rate_envelope);
    Number(doc,"spectral_step_limit_s",m.spectral_step_limit); Number(doc,"wave_step_limit_s",m.wave_step_limit);
    Number(doc,"rotary_step_limit_s",m.rotary_step_limit); Number(doc,"proposed_step_limit_s",m.proposed_step_limit);
    Number(doc,"maximum_displacement_m",ElasticShellLimits::displacement); Number(doc,"maximum_relative_energy_error",ElasticShellLimits::energy_fraction);
    Number(doc,"maximum_director_departure_rad",ElasticShellLimits::director_departure); Number(doc,"maximum_pair_angle_rad",ElasticShellLimits::pair_angle);
    Number(doc,"maximum_step_rotation_rad",ElasticShellLimits::rotation_increment); Number(doc,"maximum_membrane_shear_strain",ElasticShellLimits::strain);
    Number(doc,"maximum_thickness_curvature",ElasticShellLimits::thickness_curvature); Number(doc,"minimum_area_ratio",ElasticShellLimits::minimum_area_ratio);
    Number(doc,"maximum_area_ratio",ElasticShellLimits::maximum_area_ratio);
    String(doc,"field_phase","Endpoint shell/contact fields; reactions and applied wall impulse belong to the preceding accepted base interval");
    String(doc,"admission_scope","Restricted sampled and monitored spectral envelope; candidate geometry, energy and work gates; no general nonlinear theorem");
    AppendPenaltyAudit(doc,run);
    return doc;
}
} // namespace crash::case_data
