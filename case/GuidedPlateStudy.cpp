#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudyInternal.h"
#include "GuidedPlateContactIdentity.h"
#include "GuidedPlateExperimentProtocol.h"
#include "ShellPatchFields.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <algorithm>
#include <limits>

namespace crash::case_data {
namespace sd=study_detail;
namespace io=crash::output;
bool GuidedStudySampleEpoch(const GuidedStudyConfig& c,std::size_t j,std::uint64_t& out) noexcept {
    if(j>=kGuidedStudySamples || c.base_steps<200 || (c.refinement!=1&&c.refinement!=2&&c.refinement!=4) ||
       c.base_steps>kGuidedStudyStepCap/c.refinement) return false;
    out=((c.base_steps/200)*j+((c.base_steps%200)*j+199)/200)*c.refinement; return true;
}
bool PrepareGuidedStudyConfig(const GuidedPlateMetrics& m,const reference::ElasticCouponData& model,
                             const reference::GuidedPlateData& guided,sd::ct::Q4PlanarReferenceView view,
                             unsigned refinement,GuidedStudyConfig& output,std::string& error) {
    if(!refinement || m.required_steps%refinement || !view.parents || view.parent_count!=2 ||
       view.global_node_count!=6 || guided.wall_binding_id!=m.contact.wall_binding_id)
        return sd::Reject(error,"Study configuration has invalid refinement/reference identity");
    GuidedStudyConfig c; c.owner_id=m.stamp.owner_id; c.qualification_id=m.shell.configuration_id;c.experiment=guided.experiment;
    if(!GuidedExperimentIdentity(c.experiment,c.qualification_id)||guided.qualification_id!=c.qualification_id)
        return sd::Reject(error,"Guided Study experiment and contributor qualification disagree");
    const auto& spec=*reference::FindGuidedPlateExperiment(c.experiment);
    if(guided.stiffness_per_area!=spec.stiffness_per_area||guided.target_penetration!=spec.target_penetration||
       guided.maximum_penetration!=spec.maximum_penetration||guided.integration.force_error!=spec.force_error||
       guided.integration.energy_error!=spec.energy_error||guided.integration.max_leaves!=sd::ct::MaxQ4IntegrationLeaves||
       guided.integration.max_depth!=sd::ct::MaxQ4IntegrationDepth||guided.integration.max_visited!=sd::ct::MaxQ4IntegrationVisits)
        return sd::Reject(error,"Guided named experiment stop or integration budget differs from its immutable spec");
    c.wall_binding_id=m.contact.wall_binding_id; c.base_steps=m.required_steps/refinement; c.refinement=refinement;
    c.fixed_dt=m.stamp.fixed_dt; c.horizon=reference::GuidedPlateData::requested_horizon;
    c.initial_energy=m.initial_energy; c.wall_x=view.wall_x;
    c.integration_backend=m.contact.integration_backend;
    try {
        io::Document doc; doc.SetObject(); AppendShellReference(doc,model);
        io::Value offsets(rapidjson::kArrayType),parents(rapidjson::kArrayType),constraints(rapidjson::kArrayType);
        for(unsigned n=0;n<6;++n) {
            const auto p=model.reference_configuration.position[n]; const auto q=model.reference_configuration.rotation[n];
            c.reference_position[3*n]=p.x;c.reference_position[3*n+1]=p.y;c.reference_position[3*n+2]=p.z;
            c.reference_rotation[4*n]=q.w;c.reference_rotation[4*n+1]=q.x;c.reference_rotation[4*n+2]=q.y;c.reference_rotation[4*n+3]=q.z;
            constraints.PushBack(guided.translation_fixed_bits[n],doc.GetAllocator());
            constraints.PushBack(guided.rotation_fixed[n],doc.GetAllocator());
        }
        for(unsigned e=0;e<2;++e) {
            c.contact_reference[e]=view.parents[e];
            if(!c.contact_reference[e].covered || !sd::bounds::Nonnegative(c.contact_reference[e].area_enclosure) ||
               c.contact_reference[e].area_enclosure.lower<=0 ||
               !sd::bounds::Add(c.total_reference_area,c.contact_reference[e].area_enclosure,&c.total_reference_area))
                return sd::Reject(error,"Study requires positive exact-coordinate C3 area enclosures");
            const auto& p=c.contact_reference[e]; const auto& actual=guided.parents[e];
            if(p.parent.feature_id!=actual.feature_id || p.parent.parent_element_id!=actual.parent_element_id ||
               p.parent.parent_face_id!=actual.parent_face_id) return sd::Reject(error,"Study parent source identities disagree");
            io::Value item(rapidjson::kArrayType);
            item.PushBack(io::Value().SetUint64(p.parent.feature_id),doc.GetAllocator());
            item.PushBack(io::Value().SetUint64(p.parent.parent_element_id),doc.GetAllocator());
            item.PushBack(p.parent.parent_face_id,doc.GetAllocator());
            for(unsigned j=0;j<4;++j) {
                if(p.parent.nodes[j]!=actual.nodes[j] || p.parent.nodes[j]!=model.connectivity[e][j])
                    return sd::Reject(error,"Study parent connectivity disagrees with shell reference");
                item.PushBack(p.parent.nodes[j],doc.GetAllocator());
                const auto q=model.reference[e].node_frame_offset[j]; const double a[]{q.w,q.x,q.y,q.z};
                offsets.PushBack(io::FiniteArray(doc,a,4),doc.GetAllocator());
            }
            item.PushBack(p.area_enclosure.lower,doc.GetAllocator());item.PushBack(p.area_enclosure.upper,doc.GetAllocator());
            parents.PushBack(item,doc.GetAllocator());
        }
        doc.AddMember("frame_offsets",offsets,doc.GetAllocator());doc.AddMember("contact_parents",parents,doc.GetAllocator());
        doc.AddMember("constraints",constraints,doc.GetAllocator());
        io::Number(doc,"wall_x",c.wall_x); io::Number(doc,"contact_stiffness",guided.stiffness_per_area);
        io::Number(doc,"contact_force_error",guided.integration.force_error);
        io::Number(doc,"contact_energy_error",guided.integration.energy_error);
        // Preserve the original fingerprint byte sequence. The revised named
        // experiment additionally binds its complete stop/budget declaration;
        // these are experiment inputs, not a new numerical allowance.
        if(c.experiment!=reference::GuidedPlateExperiment::Original) {
            io::String(doc,"guided_experiment",GuidedExperimentName(c.experiment));
            io::Integer(doc,"qualification_id",c.qualification_id);
            io::Number(doc,"maximum_penetration",guided.maximum_penetration);
            io::Number(doc,"penalty_target_penetration",guided.target_penetration);
            io::Number(doc,"exposed_clearance",guided.exposed_clearance);
            io::Number(doc,"initial_gap",guided.initial_gap);io::Number(doc,"initial_tip_displacement",guided.initial_tip_displacement);
            io::Number(doc,"requested_horizon",guided.requested_horizon);
            io::Integer(doc,"contact_max_leaves",guided.integration.max_leaves);io::Integer(doc,"contact_max_depth",guided.integration.max_depth);
            io::Integer(doc,"contact_max_visits",guided.integration.max_visited);
            io::Number(doc,"maximum_displacement",ElasticShellLimits::displacement);io::Number(doc,"energy_fraction",ElasticShellLimits::energy_fraction);
            io::Number(doc,"director_departure",ElasticShellLimits::director_departure);io::Number(doc,"pair_angle",ElasticShellLimits::pair_angle);
            io::Number(doc,"rotation_increment",ElasticShellLimits::rotation_increment);io::Number(doc,"strain",ElasticShellLimits::strain);
            io::Number(doc,"thickness_curvature",ElasticShellLimits::thickness_curvature);
            io::Number(doc,"minimum_area_ratio",ElasticShellLimits::minimum_area_ratio);io::Number(doc,"maximum_area_ratio",ElasticShellLimits::maximum_area_ratio);
        }
        rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        if(!doc.Accept(writer)) return sd::Reject(error,"Study immutable serialization failed");
        c.experiment_sha256=io::Sha256({buffer.GetString(),buffer.GetSize()});
    } catch(const std::exception& e) { error=e.what(); return false; }
    if(!sd::ValidConfig(c) || m.stamp.epoch || m.stamp.time!=0 || !sd::Endpoint(c,m,error))
        return sd::Reject(error,"Invalid initial study configuration or endpoint");
    output=std::move(c);error.clear();return true;
}
namespace study_detail {
bool Certified(const GuidedStudyCertificate& c) {
    GuidedStudyCertificate checked;
    return Nonnegative(c.error)&&bounds::Certify(c.value,{c.lower,c.upper},&checked)&&checked.error<=c.error;
}
bool TimeMatches(double time,double expected,double h,std::uint64_t epoch) {
    const double roundoff=(static_cast<double>(epoch)+1)*std::numeric_limits<double>::epsilon();
    const double budget=8*roundoff*std::max({std::abs(time),std::abs(expected),h});
    return std::isfinite(time)&&std::isfinite(expected)&&std::isfinite(budget)&&roundoff<1e-3&&std::abs(time-expected)<=budget;
}
bool ValidConfig(const GuidedStudyConfig& c) {
    std::uint64_t final=0;
    if(!c.owner_id||!GuidedExperimentIdentity(c.experiment,c.qualification_id)||!c.wall_binding_id||!ValidGuidedContactBackend(c.integration_backend)||
       !GuidedStudySampleEpoch(c,200,final)||
       !std::isfinite(c.fixed_dt)||c.fixed_dt<=0||!std::isfinite(c.horizon)||c.horizon<=0||
       !std::isfinite(c.initial_energy)||c.initial_energy<=0||!std::isfinite(c.wall_x)||
       c.experiment_sha256.size()!=64||c.experiment_sha256.find_first_not_of("0123456789abcdef")!=std::string::npos||
       !TimeMatches(c.horizon,final*c.fixed_dt,c.fixed_dt,final)||
       !bounds::Nonnegative(c.total_reference_area)||c.total_reference_area.lower<=0) return false;
    for(double v:c.reference_position)if(!std::isfinite(v))return false;
    for(unsigned n=0;n<6;++n) {
        double norm=0;for(unsigned j=0;j<4;++j)norm+=c.reference_rotation[4*n+j]*c.reference_rotation[4*n+j];
        if(!std::isfinite(norm)||std::abs(norm-1)>1e-10)return false;
    }
    GuidedStudyInterval area;
    for(const auto& p:c.contact_reference) {
        if(!p.covered||!p.parent.feature_id||!p.parent.parent_element_id||p.parent.half_thickness!=0||
           !std::isfinite(p.projected_area)||p.projected_area<=0||!bounds::Nonnegative(p.area_enclosure)||
           p.area_enclosure.lower<=0||!bounds::Add(area,p.area_enclosure,&area))return false;
        for(unsigned i=0;i<4;++i) {
            if(p.parent.nodes[i]>=6)return false;
            const auto& projected=p.reference_projection[i];const auto node=p.parent.nodes[i];
            if(!ct::IsFinite(projected)||projected.x!=c.wall_x||projected.y!=c.reference_position[3*node+1]||
               projected.z!=c.reference_position[3*node+2])return false;
            for(unsigned j=0;j<i;++j)if(p.parent.nodes[i]==p.parent.nodes[j])return false;
        }
    }
    return area.lower==c.total_reference_area.lower&&area.upper==c.total_reference_area.upper;
}
bool SameStamp(const tl::fea::NodalStamp& a,const tl::fea::NodalStamp& b) {
    return tl::fea::IsCollocatedNodalTiming(a.temporal_scheme,a.velocity_phase)&&
           tl::fea::IsCollocatedNodalTiming(b.temporal_scheme,b.velocity_phase)&&a.velocity_time==b.velocity_time&&
           a.reaction_kick_dt==b.reaction_kick_dt&&
           a.owner_id==b.owner_id&&a.epoch==b.epoch&&a.node_count==b.node_count&&a.time==b.time&&a.fixed_dt==b.fixed_dt&&
           a.has_rotations==b.has_rotations&&a.reactions_valid==b.reactions_valid&&a.reaction_base_epoch==b.reaction_base_epoch&&a.reaction_time==b.reaction_time;
}
bool Force(const ct::Q4PlanarContactDiagnostics& d,GuidedStudyCertificate& out) {
    GuidedStudyInterval lower;double upper=0;
    return Nonnegative(d.wall_reaction.x)&&Nonnegative(d.force_error.x)&&bounds::Difference(d.wall_reaction.x,d.force_error.x,&lower)&&
           bounds::AddScalar(d.wall_reaction.x,d.force_error.x,true,&upper)&&
           bounds::Certify(d.wall_reaction.x,{std::max(0.,lower.lower),upper},&out);
}
bool Endpoint(const GuidedStudyConfig& c,const GuidedPlateMetrics& m,std::string& error) {
    const auto& t=m.stamp;const auto& s=m.shell;const auto& d=m.contact;
    const bool candidate=t.epoch!=0;const auto base=candidate?t.epoch-1:0;
    if(!tl::fea::IsCollocatedNodalTiming(t.temporal_scheme,t.velocity_phase)||
       t.owner_id!=c.owner_id||t.node_count!=6||!t.has_rotations||t.fixed_dt!=c.fixed_dt||
       t.epoch>c.base_steps*c.refinement||m.required_steps!=c.base_steps*c.refinement||m.initial_energy!=c.initial_energy||
       !TimeMatches(t.time,t.epoch*c.fixed_dt,c.fixed_dt,t.epoch)||
       (candidate?(!t.reactions_valid||t.reaction_base_epoch!=base||t.reaction_time+c.fixed_dt!=t.time):
                  (t.reactions_valid||t.reaction_base_epoch||t.reaction_time!=0))||
       !s.valid||!ValidGuidedContactPartition(d,c.integration_backend)||
       s.owner_id!=c.owner_id||d.owner_id!=c.owner_id||s.configuration_id!=c.qualification_id||
       d.configuration_id!=c.qualification_id||d.wall_binding_id!=c.wall_binding_id||!s.attempt||d.attempt!=s.attempt||
       s.base_epoch!=base||d.base_epoch!=base||
       s.phase!=(candidate?shell::ShellBatchPhase::kPreparedCandidate:shell::ShellBatchPhase::kAcceptedBase)||
       d.phase!=(candidate?ct::Q4PlanarContactPhase::PreparedCandidate:ct::Q4PlanarContactPhase::AcceptedBase)||
       d.parent_count!=2||d.covered_count!=2||!Certified(d.potential)||!bounds::Nonnegative(d.active_area))
        return Reject(error,"Study endpoint identity, phase or contact certificate is invalid");
    for(double x:{s.elastic_energy,s.bending_energy,s.kinetic_translation,s.kinetic_physical_rotation,s.kinetic_artificial_drilling,
                  d.maximum_penetration,m.maximum_relative_energy_error,m.peak_penetration,m.work.kinetic_energy,m.work.total_energy})
        if(!Nonnegative(x))return Reject(error,"Study endpoint energy/depth is nonfinite or negative");
    GuidedStudyCertificate force;
    const double kinetic=s.kinetic_translation+s.kinetic_physical_rotation+s.kinetic_artificial_drilling;
    if(!Force(d,force)||m.work.kinetic_energy!=kinetic||m.work.total_energy!=kinetic+s.elastic_energy+d.potential.value||
       !Nonnegative(m.wall_impulse.x))return Reject(error,"Study endpoint ledger is inconsistent");
    if(candidate) {
        const auto& a=m.applied_contact;
        if(!ValidGuidedContactPartition(a,c.integration_backend)||a.parent_count!=2||a.covered_count!=2||
           a.owner_id!=c.owner_id||a.configuration_id!=c.qualification_id||a.wall_binding_id!=c.wall_binding_id||
           a.base_epoch!=base||a.attempt!=s.attempt||a.phase!=ct::Q4PlanarContactPhase::AcceptedBase||!Force(a,force))
            return Reject(error,"Study applied force is not the consumed interval base");
    } else if(m.wall_impulse.x!=0)return Reject(error,"Initial study impulse must be zero");
    return true;
}

bool Sample(const GuidedStudyConfig& c,const GuidedPlateMetrics& m,const GuidedPlateFrame& f,
            GuidedStudySample& out,bool& partial,bool& unequal,std::string& error) {
    // Captured metrics retain the consumed interval association even if the
    // endpoint scratch below was refreshed under a newer attempt. Validate each
    // identity without equating the refresh attempt to that consumed interval.
    if(!Endpoint(c,f.metrics,error))return false;
    const auto& s=f.element_association;const auto& d=f.contact_association;
    const bool candidate=s.phase==shell::ShellBatchPhase::kPreparedCandidate;
    const auto base=candidate&&m.stamp.epoch?m.stamp.epoch-1:m.stamp.epoch;
    if(!SameStamp(f.stamp,m.stamp)||!SameStamp(f.metrics.stamp,m.stamp)||!s.valid||
       !ValidGuidedContactPartition(d,f.parent.data(),f.parent.size(),c.integration_backend)||
       d.parent_count!=2||d.covered_count!=2||!s.attempt||
       s.owner_id!=c.owner_id||d.owner_id!=c.owner_id||s.configuration_id!=c.qualification_id||d.configuration_id!=c.qualification_id||
       d.wall_binding_id!=c.wall_binding_id||s.base_epoch!=base||d.base_epoch!=base||s.attempt!=d.attempt||
       (candidate?(!m.stamp.epoch||d.phase!=ct::Q4PlanarContactPhase::PreparedCandidate):
                  (s.phase!=shell::ShellBatchPhase::kAcceptedBase||d.phase!=ct::Q4PlanarContactPhase::AcceptedBase)))
        return Reject(error,"Study capture is not a matching accepted endpoint");
    GuidedStudySample n;n.epoch=m.stamp.epoch;n.time=m.stamp.time;
    n.energy={{m.shell.kinetic_translation,m.shell.kinetic_physical_rotation,m.shell.kinetic_artificial_drilling,
               m.shell.elastic_energy,m.contact.potential.value}};
    n.shell_bending_energy=m.shell.bending_energy;n.maximum_penetration=m.contact.maximum_penetration;n.contact_potential=m.contact.potential;
    if(!Force(m.contact,n.normal_wall_force))return Reject(error,"Study sample force certificate overflow");
    double displacement[6]{},rotation[6]{};
    n.minimum_signed_gap={std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
    for(unsigned i=0;i<6;++i) {
        for(unsigned j=0;j<3;++j)if(!std::isfinite(f.position[3*i+j])||!std::isfinite(f.velocity[3*i+j]))
            return Reject(error,"Study sample geometry/velocity is nonfinite");
        displacement[i]=f.position[3*i]-c.reference_position[3*i];
        GuidedStudyInterval gap;if(!bounds::Difference(c.wall_x,f.position[3*i],&gap))return Reject(error,"Study signed gap overflow");
        n.minimum_signed_gap.lower=std::min(n.minimum_signed_gap.lower,gap.lower);
        n.minimum_signed_gap.upper=std::min(n.minimum_signed_gap.upper,gap.upper);
        chrono::ChQuaterniond q(f.rotation[4*i],f.rotation[4*i+1],f.rotation[4*i+2],f.rotation[4*i+3]);
        const chrono::ChQuaterniond q0(c.reference_rotation[4*i],c.reference_rotation[4*i+1],c.reference_rotation[4*i+2],c.reference_rotation[4*i+3]);
        if(!std::isfinite(q.Length2())||std::abs(q.Length2()-1)>1e-10)return Reject(error,"Study orientation is not finite and unit");
        auto relative=q*q0.GetConjugate();if(relative.e0()<0)relative=-relative;
        const auto spin=relative.GetRotVec();
        if(!std::isfinite(spin.Length2())||spin.Length()>=.25)return Reject(error,"Study orientation is outside the admitted observation chart");
        rotation[i]=spin.z();
    }
    n.normal_displacement={{.5*displacement[4]+.5*displacement[5],.5*displacement[0]+.5*displacement[3]}};
    n.normal_velocity={{.5*f.velocity[12]+.5*f.velocity[15],.5*f.velocity[0]+.5*f.velocity[9]}};
    n.world_z_rotation={{.5*rotation[4]+.5*rotation[5],.5*rotation[0]+.5*rotation[3]}};
    n.curvature_proxy=(n.normal_displacement[0]-2*n.normal_displacement[1]+.5*displacement[1]+.5*displacement[2])/(.1*.1);
    if(!bounds::Add({f.velocity[12],f.velocity[12]},{f.velocity[15],f.velocity[15]},&n.tip_normal_velocity)||
       !bounds::Scale(n.tip_normal_velocity,.5,&n.tip_normal_velocity)||!std::isfinite(n.curvature_proxy))
        return Reject(error,"Study nodal observable overflow");
    bool p=false,u=false;
    for(unsigned e=0;e<2;++e) {
        const auto& saved=c.contact_reference[e];const auto& r=f.parent[e].integration;
        if(!f.parent[e].covered||r.parent_element_id!=saved.parent.parent_element_id||r.feature_id!=saved.parent.feature_id||
           r.base_epoch!=base||r.attempt!=s.attempt||!bounds::Nonnegative(r.active_area))
            return Reject(error,"Study parent field identity/area is invalid");
        p=p||(r.active_area.lower>0&&r.active_area.upper<saved.area_enclosure.lower);
        for(unsigned i=0;i<4;++i) {
            if(!Certified(r.force[i]))return Reject(error,"Study nodal force certificate is invalid");
            for(unsigned j=0;j<i;++j)u=u||r.force[i].lower>r.force[j].upper||r.force[j].lower>r.force[i].upper;
        }
    }
    out=n;partial=p;unequal=u;return true;
}
} // namespace study_detail
} // namespace crash::case_data
