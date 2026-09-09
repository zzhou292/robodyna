#include "GuidedPlateStudyIO.h"
#include "GuidedPlateContactProtocol.h"
#include "GuidedPlateExperimentProtocol.h"
#include "output/ArtifactIO.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace crash::case_data {
namespace {
namespace io=output;
using io::Value;
const Value& Member(const Value& object,const char* name) {
    io::Require(object.IsObject(),"Study field must belong to an object");
    const Value* found=nullptr;
    for(auto i=object.MemberBegin();i!=object.MemberEnd();++i)if(i->name==name) {
        io::Require(!found,"Duplicate guided study field");found=&i->value;
    }
    io::Require(found,"Required guided study field is missing");return *found;
}
double Number(const Value& v) {
    io::Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Study number must be finite");return v.GetDouble();
}
double Number(const Value& v,const char* name) {return Number(Member(v,name));}
std::uint64_t Integer(const Value& v,const char* name) {
    const auto& a=Member(v,name);io::Require(a.IsUint64(),"Study integer must be uint64");return a.GetUint64();
}
std::uint32_t Index(const Value& v) {
    io::Require(v.IsUint(),"Study index must be uint32");return v.GetUint();
}
bool Boolean(const Value& v,const char* name) {
    const auto& a=Member(v,name);io::Require(a.IsBool(),"Study flag must be boolean");return a.GetBool();
}
std::string String(const Value& v,const char* name) {
    const auto& a=Member(v,name);io::Require(a.IsString(),"Study text must be a string");return {a.GetString(),a.GetStringLength()};
}
const Value& Array(const Value& v,const char* name,std::size_t size) {
    const auto& a=Member(v,name);io::Require(a.IsArray()&&a.Size()==size,"Study array shape differs from schema");return a;
}
template<std::size_t N> std::array<double,N> Doubles(const Value& v,const char* key) {
    const auto& a=Array(v,key,N);std::array<double,N> out{};
    for(std::size_t i=0;i<N;++i)out[i]=Number(a[static_cast<rapidjson::SizeType>(i)]);return out;
}
GuidedStudyInterval Interval(const Value& v,const char* key) {
    auto a=Doubles<2>(v,key);return {a[0],a[1]};
}
GuidedStudyCertificate Certificate(const Value& v,const char* key) {
    auto a=Doubles<4>(v,key);GuidedStudyCertificate c;c.value=a[0];c.lower=a[1];c.upper=a[2];c.error=a[3];return c;
}
GuidedStudyEvent Event(const Value& v,const char* key) {
    const auto& a=Member(v,key);GuidedStudyEvent e;e.observed=Boolean(a,"observed");
    e.lower_epoch=Integer(a,"lower_epoch");e.upper_epoch=Integer(a,"upper_epoch");
    e.lower_time=Number(a,"lower_time");e.upper_time=Number(a,"upper_time");return e;
}
GuidedStudyConfig Config(const Value& v) {
    GuidedStudyConfig c;c.owner_id=Integer(v,"owner_id");c.qualification_id=Integer(v,"qualification_id");
    io::Require(ParseGuidedExperiment(io::guided_experiment_metadata::Name(v),c.experiment)&&
        GuidedExperimentIdentity(c.experiment,c.qualification_id),"Guided experiment name and qualification disagree");
    c.wall_binding_id=Integer(v,"wall_binding_id");c.base_steps=Integer(v,"base_steps");c.refinement=Index(Member(v,"refinement"));
    c.fixed_dt=Number(v,"fixed_dt");c.horizon=Number(v,"horizon");c.initial_energy=Number(v,"initial_energy");
    io::Require(ParseGuidedContactBackend(io::contact_metadata::Backend(v),c.integration_backend),"Unknown guided contact integration backend");
    c.wall_x=Number(v,"wall_x");c.experiment_sha256=String(v,"experiment_sha256");
    c.reference_position=Doubles<18>(v,"reference_position");c.reference_rotation=Doubles<24>(v,"reference_rotation");
    c.total_reference_area=Interval(v,"total_reference_area");const auto& parents=Array(v,"contact_reference",2);
    for(unsigned e=0;e<2;++e) {
        auto& p=c.contact_reference[e];const auto& a=parents[e];p.parent.feature_id=Integer(a,"feature_id");
        p.parent.parent_element_id=Integer(a,"parent_element_id");
        p.parent.parent_face_id=Index(Member(a,"parent_face_id"));p.covered=Boolean(a,"covered");
        p.parent.half_thickness=Number(a,"half_thickness");
        const auto& nodes=Array(a,"nodes",4);for(unsigned n=0;n<4;++n)p.parent.nodes[n]=Index(nodes[n]);
        const auto xyz=Doubles<12>(a,"reference_projection");
        for(unsigned n=0;n<4;++n)p.reference_projection[n]={xyz[3*n],xyz[3*n+1],xyz[3*n+2]};
        p.projected_area=Number(a,"projected_area");p.area_enclosure=Interval(a,"area_enclosure");
    }
    return c;
}
GuidedStudySummary Summary(const Value& v) {
    GuidedStudySummary s;s.accepted_epoch=Integer(v,"accepted_epoch");s.last_attempt=Integer(v,"last_attempt");
    s.accepted_time=Number(v,"accepted_time");const auto count=Integer(v,"sample_count");
    io::Require(count==kGuidedStudySamples,"Completed study requires all 201 samples");s.sample_count=count;
    s.sampled_peak_normal_force=Certificate(v,"sampled_peak_normal_force");s.normal_wall_impulse=Certificate(v,"normal_wall_impulse");
    s.maximum_penetration=Number(v,"maximum_penetration");
    s.maximum_value_energy_relative_error=Number(v,"maximum_value_energy_relative_error");
    s.maximum_certified_energy_relative_error=Number(v,"maximum_certified_energy_relative_error");
    s.activation=Event(v,"activation");s.pressure_release=Event(v,"pressure_release");
    s.certified_partial_area=Boolean(v,"certified_partial_area");s.certified_unequal_nodal_force=Boolean(v,"certified_unequal_nodal_force");
    s.separated_rebounding=Boolean(v,"separated_rebounding");s.separation_sample_epoch=Integer(v,"separation_sample_epoch");
    s.separation_sample_time=Number(v,"separation_sample_time");s.minimum_curvature=Number(v,"minimum_curvature");
    s.maximum_curvature=Number(v,"maximum_curvature");s.maximum_abs_curvature=Number(v,"maximum_abs_curvature");
    s.minimum_rotation=Doubles<2>(v,"minimum_rotation");s.maximum_rotation=Doubles<2>(v,"maximum_rotation");return s;
}
GuidedStudySample Sample(const Value& v) {
    GuidedStudySample s;s.epoch=Integer(v,"epoch");s.time=Number(v,"time");
    s.normal_displacement=Doubles<2>(v,"normal_displacement");s.normal_velocity=Doubles<2>(v,"normal_velocity");
    s.world_z_rotation=Doubles<2>(v,"world_z_rotation");s.curvature_proxy=Number(v,"curvature_proxy");
    s.minimum_signed_gap=Interval(v,"minimum_signed_gap");s.tip_normal_velocity=Interval(v,"tip_normal_velocity");
    s.energy=Doubles<static_cast<std::size_t>(GuidedStudyEnergy::Count)>(v,"energy");
    s.shell_bending_energy=Number(v,"shell_bending_energy");s.maximum_penetration=Number(v,"maximum_penetration");
    s.normal_wall_force=Certificate(v,"normal_wall_force");s.contact_potential=Certificate(v,"contact_potential");return s;
}
} // namespace
GuidedStudyData ReadGuidedPlateStudy(const std::filesystem::path& path) {
    return ParseGuidedPlateStudy(io::ReadBounded(path,kGuidedStudyByteCap));
}
GuidedStudyData ParseGuidedPlateStudy(const std::string& bytes) {
    io::Require(bytes.size()<=kGuidedStudyByteCap,"Guided study exceeds 1 MiB report cap");io::Document d;
    d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
    io::Require(!d.HasParseError()&&d.IsObject(),"Guided study JSON parse failed");
    io::Require(String(d,"schema")=="robo_dyna.guided_plate_study.v1","Unknown guided study schema");
    io::Require(String(d,"energy_order")=="translation,physical_rotation,artificial_drilling,shell,contact"&&
                String(d,"sample_schedule")=="ceil(j*base_steps/200)*refinement; j=0..200"&&
                String(d,"certificate_order")=="value,lower,upper,error; value may lie outside truth interval",
                "Guided study field conventions disagree with schema");
    GuidedStudyData out;out.complete=Boolean(d,"complete");out.config=Config(Member(d,"configuration"));
    out.summary=Summary(Member(d,"summary"));out.initial_position=Doubles<18>(d,"initial_position");
    out.initial_rotation=Doubles<24>(d,"initial_rotation");const auto& samples=Array(d,"samples",kGuidedStudySamples);
    for(unsigned i=0;i<kGuidedStudySamples;++i)out.samples[i]=Sample(samples[i]);
    std::string error;if(!ValidateGuidedPlateStudy(out,error))throw std::runtime_error(error);return out;
}
} // namespace crash::case_data
