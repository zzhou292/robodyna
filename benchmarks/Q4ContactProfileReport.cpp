#include "Q4ContactProfile.h"
#include "output/ArtifactIO.h"

namespace crash::profile {
namespace {
namespace io=output;
void Counts(io::Document& d,const char* name,const std::array<std::uint32_t,3>& value,std::uint32_t total) {
    io::Require(value[0]+value[1]+value[2]==total,"Invalid profile leaf-kind total");
    io::Value counts(rapidjson::kArrayType);for(auto count:value)counts.PushBack(count,d.GetAllocator());
    d.AddMember(io::Value(name,d.GetAllocator()),counts,d.GetAllocator());
}
void Certificates(io::Document& d,const contact::Q4IntegrationResult& r) {
    io::Value forces(rapidjson::kArrayType),nodal(rapidjson::kArrayType),couples(rapidjson::kArrayType);
    for(unsigned n=0;n<4;++n) {
        const auto& f=r.force[n];const double force[]{f.value,f.lower,f.upper,f.error};
        const auto& v=r.nodal.forces[n];const double xyz[]{v.x,v.y,v.z};
        const auto& c=r.nodal.couples[n];const double moment[]{c.x,c.y,c.z};
        forces.PushBack(io::FiniteArray(d,force,4),d.GetAllocator());
        nodal.PushBack(io::FiniteArray(d,xyz,3),d.GetAllocator());couples.PushBack(io::FiniteArray(d,moment,3),d.GetAllocator());
    }
    d.AddMember("nodal_magnitude_N",forces,d.GetAllocator());d.AddMember("nodal_force_xyz_N",nodal,d.GetAllocator());
    d.AddMember("nodal_couple_xyz_Nm",couples,d.GetAllocator());
    const double active[]{r.active_area.lower,r.active_area.upper};io::FiniteArray(d,"active_area_m2",active,2);
}
void ScalarReference(io::Document& d,const ContactParentProfile& profile) {
    io::Require(profile.scalar_comparison&&profile.scalar_report.status==contact::Q4IntegrationStatus::Ok,
                "Rectangular profile lacks checked scalar reference");
    const auto& r=profile.scalar_result;io::Document scalar;scalar.SetObject();
    io::String(scalar,"backend","scalar-dyadic-squares");io::Number(scalar,"host_ms",profile.scalar_host_ms);
    io::Integer(scalar,"leaf_count",r.leaf_count);io::Integer(scalar,"visited",r.visited);io::Integer(scalar,"deepest_leaf",r.deepest_leaf);
    Counts(scalar,"leaf_kinds_inactive_active_mixed",profile.scalar_leaf_kinds,r.leaf_count);
    const double force[]{r.resultant.value,r.resultant.lower,r.resultant.upper,r.resultant.error};io::FiniteArray(scalar,"resultant_N",force,4);
    const double energy[]{r.potential.value,r.potential.lower,r.potential.upper,r.potential.error};io::FiniteArray(scalar,"potential_J",energy,4);
    Certificates(scalar,r);
    io::FiniteArray(scalar,"candidate_absolute_difference_upper",profile.scalar_difference_upper.data(),6);
    io::FiniteArray(scalar,"combined_certificate_error_upper",profile.scalar_error_sum_upper.data(),6);
    io::String(scalar,"comparison_columns","nodal0_N,nodal1_N,nodal2_N,nodal3_N,resultant_N,potential_J");
    io::Boolean(scalar,"certificates_overlap_and_estimates_compatible",true);
    io::String(scalar,"comparison_scope","Same supplied bilinear field and absolute budgets; different partitions and quadrature estimates allowed. No dynamics or convergence claim.");
    io::Value copied;copied.CopyFrom(scalar,d.GetAllocator());d.AddMember("scalar_cpu_reference",copied,d.GetAllocator());
}
} // namespace
void WriteContactProfile(const std::string& path,const ContactProfileSource& source,const ContactProfileResult& result) {
    namespace io=output;io::Document d;d.SetObject();
    io::Require(result.backend==ContactProfileBackend::Scalar||result.backend==ContactProfileBackend::Rectangular,"Invalid profile backend");
    const bool rectangular=result.backend==ContactProfileBackend::Rectangular;
    io::String(d,"schema",rectangular?"robo_dyna.q4_contact_rectangular_profile.v1":"robo_dyna.q4_contact_profile.v1");
    io::String(d,"scope",rectangular?
        "Prescribed saved x/v with original C3 setup; qualification rectangular C2 CPU/single-thread CUDA parity and scalar CPU certificate comparison. No dynamics/restart, geometry snap or capacity/tolerance changes.":
        "Prescribed saved x/v with original C3 setup; existing C2 CPU/single-thread CUDA parity and timing. No dynamics/restart or capacity/tolerance changes.");
    io::String(d,"backend",rectangular?"qualification-dyadic-rectangles":"scalar-dyadic-squares");
    io::Integer(d,"owned_scratch_bytes",result.scratch_bytes);
    io::String(d,"memory_scope","Selected backend only; explicit_device_bytes includes input/result/heap/leaves. Driver context/module/stack memory is additional.");
    io::String(d,"depth_scope",rectangular?"Independent U/V depths; deepest_leaf is their maximum, not binary tree path length":"Square depth; U and V depths are equal");
    io::String(d,"finish_timing_scope","Repeated finalization of the retained partition, not instrumented attribution inside the first integration call");
    io::String(d,"leaf_kind_scope","Exact retained CPU partition counts; counted outside host/kernel integration timers");
    io::String(d,"frame_sha256",source.frame_sha256);io::String(d,"canonical_wall_sha256",source.wall_sha256);
    const auto& input=source.input;io::Integer(d,"saved_accepted_epoch",input.epoch);io::Integer(d,"attempt",input.attempt);
    io::Number(d,"saved_accepted_time_s",source.accepted_time);io::Integer(d,"explicit_device_bytes",result.device_bytes);
    io::Number(d,"force_error_N",input.limits.force_error);io::Number(d,"energy_error_J",input.limits.energy_error);
    io::Number(d,"wall_x_m",input.wall_x);io::Number(d,"stiffness_N_per_m3",input.stiffness);
    io::Number(d,"maximum_penetration_m",input.maximum_penetration);io::Integer(d,"max_leaves",input.limits.max_leaves);
    io::Integer(d,"max_visited",input.limits.max_visited);io::Integer(d,"max_depth",input.limits.max_depth);
    io::FiniteArray(d,"position_xyz_m",input.position,18);io::FiniteArray(d,"velocity_xyz_m_per_s",input.velocity,18);
    io::FiniteArray(d,"inverse_mass",input.inverse_mass,6);
    io::Value masks(rapidjson::kArrayType);for(auto m:input.fixed)masks.PushBack(m,d.GetAllocator());d.AddMember("fixed_bits",masks,d.GetAllocator());
    io::Value parents(rapidjson::kArrayType);
    for(unsigned p=0;p<2;++p) {
        const auto& data=result.parents[p];const auto& r=data.result;io::Document item;item.SetObject();
        io::Require(data.report.status==contact::Q4IntegrationStatus::Ok&&r.valid,"Profile parent is not successfully integrated");
        io::Integer(item,"feature_id",r.feature_id);io::Integer(item,"parent_element_id",r.parent_element_id);
        io::Integer(item,"parent_face_id",input.parents[p].parent_face_id);io::Number(item,"projected_area_m2",input.area[p]);
        io::Value nodes(rapidjson::kArrayType);for(auto n:input.parents[p].nodes)nodes.PushBack(n,item.GetAllocator());item.AddMember("nodes",nodes,item.GetAllocator());
        io::Number(item,"host_ms",data.host_ms);double gpu[]{data.gpu_ms[0],data.gpu_ms[1],data.gpu_ms[2]};io::FiniteArray(item,"cuda_event_ms",gpu,3);
        io::Number(item,"host_finish_ms",data.host_finish_ms);
        double finish[]{data.gpu_finish_ms[0],data.gpu_finish_ms[1],data.gpu_finish_ms[2]};io::FiniteArray(item,"cuda_finish_event_ms",finish,3);
        io::Integer(item,"leaf_count",r.leaf_count);io::Integer(item,"visited",r.visited);io::Integer(item,"deepest_leaf",r.deepest_leaf);
        io::Integer(item,"deepest_u",data.deepest_u);io::Integer(item,"deepest_v",data.deepest_v);
        Counts(item,"leaf_kinds_inactive_active_mixed",data.leaf_kinds,r.leaf_count);
        const double force[]{r.resultant.value,r.resultant.lower,r.resultant.upper,r.resultant.error};io::FiniteArray(item,"resultant_N",force,4);
        const double energy[]{r.potential.value,r.potential.lower,r.potential.upper,r.potential.error};io::FiniteArray(item,"potential_J",energy,4);
        io::Boolean(item,"all_cpu_cuda_results_equal",true);
        Certificates(item,r);
        if(rectangular)ScalarReference(item,data);
        io::Value copy;copy.CopyFrom(item,d.GetAllocator());parents.PushBack(copy,d.GetAllocator());
    }
    d.AddMember("parents",parents,d.GetAllocator());io::WriteJson(path,d);
}
} // namespace crash::profile
