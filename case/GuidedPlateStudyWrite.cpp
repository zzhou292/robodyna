#include "GuidedPlateStudyIO.h"
#include "output/ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include <stdexcept>

namespace crash::case_data {
namespace {
namespace io=output;
using io::Document;
void Interval(Document& d,const char* key,const GuidedStudyInterval& a) {
    const double v[]{a.lower,a.upper};io::FiniteArray(d,key,v,2);
}
void Certificate(Document& d,const char* key,const GuidedStudyCertificate& a) {
    const double v[]{a.value,a.lower,a.upper,a.error};io::FiniteArray(d,key,v,4);
}
template<std::size_t N> void Array(Document& d,const char* key,const std::array<double,N>& a) {
    io::FiniteArray(d,key,a.data(),a.size());
}
void Child(Document& d,const char* key,Document& child) {
    io::Value value;value.CopyFrom(child,d.GetAllocator());
    d.AddMember(io::Value(key,d.GetAllocator()),value,d.GetAllocator());
}
void Event(Document& d,const char* key,const GuidedStudyEvent& e) {
    Document child;child.SetObject();io::Boolean(child,"observed",e.observed);
    io::Integer(child,"lower_epoch",e.lower_epoch);io::Integer(child,"upper_epoch",e.upper_epoch);
    io::Number(child,"lower_time",e.lower_time);io::Number(child,"upper_time",e.upper_time);Child(d,key,child);
}
Document Config(const GuidedStudyConfig& c) {
    Document d;d.SetObject();
    io::Integer(d,"owner_id",c.owner_id);io::Integer(d,"qualification_id",c.qualification_id);
    io::Integer(d,"wall_binding_id",c.wall_binding_id);io::Integer(d,"base_steps",c.base_steps);
    io::Integer(d,"refinement",c.refinement);io::Number(d,"fixed_dt",c.fixed_dt);io::Number(d,"horizon",c.horizon);
    io::Number(d,"initial_energy",c.initial_energy);io::Number(d,"wall_x",c.wall_x);
    io::String(d,"experiment_sha256",c.experiment_sha256);
    Array(d,"reference_position",c.reference_position);Array(d,"reference_rotation",c.reference_rotation);
    Interval(d,"total_reference_area",c.total_reference_area);
    io::Value parents(rapidjson::kArrayType);
    for(const auto& p:c.contact_reference) {
        Document item;item.SetObject();io::Integer(item,"feature_id",p.parent.feature_id);
        io::Integer(item,"parent_element_id",p.parent.parent_element_id);
        io::Integer(item,"parent_face_id",p.parent.parent_face_id);io::Boolean(item,"covered",p.covered);
        io::Number(item,"half_thickness",p.parent.half_thickness);
        io::Value nodes(rapidjson::kArrayType);for(auto n:p.parent.nodes)nodes.PushBack(n,item.GetAllocator());
        item.AddMember("nodes",nodes,item.GetAllocator());
        double xyz[12];for(unsigned n=0;n<4;++n) {
            xyz[3*n]=p.reference_projection[n].x;xyz[3*n+1]=p.reference_projection[n].y;xyz[3*n+2]=p.reference_projection[n].z;
        }
        io::FiniteArray(item,"reference_projection",xyz,12);io::Number(item,"projected_area",p.projected_area);
        Interval(item,"area_enclosure",p.area_enclosure);
        io::Value copy;copy.CopyFrom(item,d.GetAllocator());parents.PushBack(copy,d.GetAllocator());
    }
    d.AddMember("contact_reference",parents,d.GetAllocator());return d;
}
Document Summary(const GuidedStudySummary& s) {
    Document d;d.SetObject();io::Integer(d,"accepted_epoch",s.accepted_epoch);io::Integer(d,"last_attempt",s.last_attempt);
    io::Number(d,"accepted_time",s.accepted_time);io::Integer(d,"sample_count",s.sample_count);
    Certificate(d,"sampled_peak_normal_force",s.sampled_peak_normal_force);Certificate(d,"normal_wall_impulse",s.normal_wall_impulse);
    io::Number(d,"maximum_penetration",s.maximum_penetration);
    io::Number(d,"maximum_value_energy_relative_error",s.maximum_value_energy_relative_error);
    io::Number(d,"maximum_certified_energy_relative_error",s.maximum_certified_energy_relative_error);
    Event(d,"activation",s.activation);Event(d,"pressure_release",s.pressure_release);
    io::Boolean(d,"certified_partial_area",s.certified_partial_area);
    io::Boolean(d,"certified_unequal_nodal_force",s.certified_unequal_nodal_force);
    io::Boolean(d,"separated_rebounding",s.separated_rebounding);
    io::Integer(d,"separation_sample_epoch",s.separation_sample_epoch);io::Number(d,"separation_sample_time",s.separation_sample_time);
    io::Number(d,"minimum_curvature",s.minimum_curvature);io::Number(d,"maximum_curvature",s.maximum_curvature);
    io::Number(d,"maximum_abs_curvature",s.maximum_abs_curvature);
    Array(d,"minimum_rotation",s.minimum_rotation);Array(d,"maximum_rotation",s.maximum_rotation);return d;
}
Document Sample(const GuidedStudySample& s) {
    Document d;d.SetObject();io::Integer(d,"epoch",s.epoch);io::Number(d,"time",s.time);
    Array(d,"normal_displacement",s.normal_displacement);Array(d,"normal_velocity",s.normal_velocity);
    Array(d,"world_z_rotation",s.world_z_rotation);io::Number(d,"curvature_proxy",s.curvature_proxy);
    Interval(d,"minimum_signed_gap",s.minimum_signed_gap);Interval(d,"tip_normal_velocity",s.tip_normal_velocity);
    Array(d,"energy",s.energy);io::Number(d,"shell_bending_energy",s.shell_bending_energy);
    io::Number(d,"maximum_penetration",s.maximum_penetration);
    Certificate(d,"normal_wall_force",s.normal_wall_force);Certificate(d,"contact_potential",s.contact_potential);return d;
}
void WriteBounded(const std::filesystem::path& path,const Document& doc) {
    rapidjson::StringBuffer buffer;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    io::Require(doc.Accept(writer),"Guided study serialization failed");
    io::Require(buffer.GetSize()<kGuidedStudyByteCap,"Guided study exceeds 1 MiB report cap");
    io::WriteBytes(path,std::string(buffer.GetString(),buffer.GetSize())+"\n");
}
} // namespace
void WriteGuidedPlateStudy(const std::filesystem::path& path,const GuidedStudyData& data) {
    std::string error;if(!ValidateGuidedPlateStudy(data,error))throw std::runtime_error(error);
    Document d;d.SetObject();io::String(d,"schema","robo_dyna.guided_plate_study.v1");
    io::Boolean(d,"complete",data.complete);
    auto config=Config(data.config);Child(d,"configuration",config);auto summary=Summary(data.summary);Child(d,"summary",summary);
    Array(d,"initial_position",data.initial_position);Array(d,"initial_rotation",data.initial_rotation);
    io::String(d,"energy_order","translation,physical_rotation,artificial_drilling,shell,contact");
    io::String(d,"sample_schedule","ceil(j*base_steps/200)*refinement; j=0..200");
    io::String(d,"certificate_order","value,lower,upper,error; value may lie outside truth interval");
    io::Value samples(rapidjson::kArrayType);
    for(const auto& s:data.samples) {auto item=Sample(s);io::Value copy;copy.CopyFrom(item,d.GetAllocator());samples.PushBack(copy,d.GetAllocator());}
    d.AddMember("samples",samples,d.GetAllocator());WriteBounded(path,d);
}
void WriteGuidedPlateComparison(const std::filesystem::path& path,const GuidedStudyComparison& c,
                               const std::string& coarse,const std::string& fine) {
    for(const auto* hash:{&coarse,&fine})io::Require(hash->size()==64&&hash->find_first_not_of("0123456789abcdef")==std::string::npos,
                                                 "Comparison requires report SHA256 bindings");
    Document d;d.SetObject();io::String(d,"schema","robo_dyna.guided_plate_comparison.v1");
    io::String(d,"coarse_sha256",coarse);io::String(d,"fine_sha256",fine);io::Boolean(d,"passed",c.passed);
    io::String(d,"diagnostic",c.diagnostic);
    io::Number(d,"displacement_ratio",c.displacement_ratio);io::Number(d,"velocity_ratio",c.velocity_ratio);
    io::Number(d,"rotation_ratio",c.rotation_ratio);io::Number(d,"force_ratio",c.force_ratio);
    io::Number(d,"impulse_ratio",c.impulse_ratio);io::Number(d,"energy_ratio",c.energy_ratio);
    io::Number(d,"event_ratio",c.event_ratio);io::Number(d,"penetration_ratio",c.penetration_ratio);
    io::Boolean(d,"energy_envelopes",c.energy_envelopes);io::Boolean(d,"deforming_contact_evidence",c.deforming_contact_evidence);
    io::Boolean(d,"events_complete",c.events_complete);WriteBounded(path,d);
}
} // namespace crash::case_data
