#include "GuidedPlateStudyInternal.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <limits>

namespace crash::case_data {
namespace sd=study_detail;
namespace {
template<std::size_t N> bool SameBits(const std::array<double,N>& a,const std::array<double,N>& b) {
    for(std::size_t i=0;i<N;++i)if(output::Bits(a[i])!=output::Bits(b[i]))return false;
    return true;
}
bool EventValid(const GuidedStudyEvent& e,const GuidedStudyConfig& c) {
    return !e.observed || (e.lower_epoch<e.upper_epoch&&e.upper_epoch<=c.base_steps*c.refinement&&
        sd::TimeMatches(e.lower_time,e.lower_epoch*c.fixed_dt,c.fixed_dt,e.lower_epoch)&&
        sd::TimeMatches(e.upper_time,e.upper_epoch*c.fixed_dt,c.fixed_dt,e.upper_epoch));
}
bool Valid(const GuidedStudyData& d) {
    const auto& c=d.config;const auto& s=d.summary;
    if(!d.complete||!sd::ValidConfig(c)||s.sample_count!=kGuidedStudySamples||s.accepted_epoch!=c.base_steps*c.refinement||
       !sd::TimeMatches(s.accepted_time,c.horizon,c.fixed_dt,s.accepted_epoch)||!s.last_attempt||
       !sd::Certified(s.sampled_peak_normal_force)||!sd::Certified(s.normal_wall_impulse)||
       !EventValid(s.activation,c)||!EventValid(s.pressure_release,c)||
       (s.pressure_release.observed&&(!s.activation.observed||s.pressure_release.lower_epoch<s.activation.upper_epoch)))return false;
    for(double x:{s.maximum_penetration,s.maximum_value_energy_relative_error,s.maximum_certified_energy_relative_error,s.maximum_abs_curvature})
        if(!sd::Nonnegative(x))return false;
    if(s.maximum_certified_energy_relative_error<s.maximum_value_energy_relative_error||
       !std::isfinite(s.minimum_curvature)||!std::isfinite(s.maximum_curvature)||s.maximum_curvature<s.minimum_curvature)return false;
    for(unsigned i=0;i<2;++i)if(!std::isfinite(s.minimum_rotation[i])||!std::isfinite(s.maximum_rotation[i])||
                               s.minimum_rotation[i]>s.maximum_rotation[i])return false;
    for(double x:d.initial_position)if(!std::isfinite(x))return false;
    for(double x:d.initial_rotation)if(!std::isfinite(x))return false;
    bool separation_verified=false;
    double minimum_curvature=d.samples[0].curvature_proxy,maximum_curvature=minimum_curvature,maximum_abs_curvature=0;
    auto minimum_rotation=d.samples[0].world_z_rotation,maximum_rotation=minimum_rotation;
    for(std::size_t i=0;i<kGuidedStudySamples;++i) {
        const auto& a=d.samples[i];std::uint64_t epoch=0;
        if(!GuidedStudySampleEpoch(c,i,epoch)||a.epoch!=epoch||!sd::TimeMatches(a.time,a.epoch*c.fixed_dt,c.fixed_dt,a.epoch)||
           !sd::Certified(a.normal_wall_force)||!sd::Certified(a.contact_potential)||!sd::bounds::Finite(a.minimum_signed_gap)||
           !sd::bounds::Finite(a.tip_normal_velocity)||!sd::Nonnegative(a.shell_bending_energy)||
           !sd::Nonnegative(a.maximum_penetration)||!std::isfinite(a.curvature_proxy)||
           a.energy[4]!=a.contact_potential.value||a.maximum_penetration>s.maximum_penetration||
           a.normal_wall_force.value>s.sampled_peak_normal_force.value||a.normal_wall_force.lower>s.sampled_peak_normal_force.lower||
           a.normal_wall_force.upper>s.sampled_peak_normal_force.upper)return false;
        for(unsigned j=0;j<2;++j)if(!std::isfinite(a.normal_displacement[j])||!std::isfinite(a.normal_velocity[j])||
                                   !std::isfinite(a.world_z_rotation[j]))return false;
        for(double x:a.energy)if(!sd::Nonnegative(x))return false;
        minimum_curvature=std::min(minimum_curvature,a.curvature_proxy);
        maximum_curvature=std::max(maximum_curvature,a.curvature_proxy);
        maximum_abs_curvature=std::max(maximum_abs_curvature,std::abs(a.curvature_proxy));
        for(unsigned j=0;j<2;++j) {
            minimum_rotation[j]=std::min(minimum_rotation[j],a.world_z_rotation[j]);
            maximum_rotation[j]=std::max(maximum_rotation[j],a.world_z_rotation[j]);
        }
        const double total=a.energy[0]+a.energy[1]+a.energy[2]+a.energy[3]+a.energy[4];
        double difference=0,with_uncertainty=0;GuidedStudyInterval relative;
        if(!sd::bounds::AbsoluteDifferenceUpper(total,c.initial_energy,&difference)||
           !sd::bounds::AddScalar(difference,a.contact_potential.error,true,&with_uncertainty)||
           !sd::bounds::DividePositive({with_uncertainty,with_uncertainty},c.initial_energy,&relative)||
           relative.upper>s.maximum_certified_energy_relative_error||
           std::abs(total-c.initial_energy)/c.initial_energy>s.maximum_value_energy_relative_error)return false;
        if(s.separated_rebounding&&a.epoch==s.separation_sample_epoch)
            separation_verified=s.pressure_release.observed&&a.epoch>=s.pressure_release.upper_epoch&&
                a.time==s.separation_sample_time&&a.contact_potential.upper==0&&a.minimum_signed_gap.lower>0&&a.tip_normal_velocity.upper<0;
    }
    // These extrema depend only on the retained common samples. Per-interval
    // peaks above may exceed sample maxima, but cannot understate any sample.
    return minimum_curvature==s.minimum_curvature&&maximum_curvature==s.maximum_curvature&&
        maximum_abs_curvature==s.maximum_abs_curvature&&SameBits(minimum_rotation,s.minimum_rotation)&&
        SameBits(maximum_rotation,s.maximum_rotation)&&(!s.separated_rebounding||separation_verified);
}
bool SamePhysicalExperiment(const GuidedStudyData& a,const GuidedStudyData& b) {
    const auto& c=a.config;const auto& f=b.config;
    if(c.experiment!=f.experiment||c.qualification_id!=f.qualification_id||c.experiment_sha256!=f.experiment_sha256||c.horizon!=f.horizon||
       c.initial_energy!=f.initial_energy||output::Bits(c.wall_x)!=output::Bits(f.wall_x)||
       !SameBits(c.reference_position,f.reference_position)||!SameBits(c.reference_rotation,f.reference_rotation)||
       !SameBits(a.initial_position,b.initial_position)||!SameBits(a.initial_rotation,b.initial_rotation))return false;
    for(unsigned i=0;i<2;++i) {
        const auto& x=c.contact_reference[i];const auto& y=f.contact_reference[i];
        if(x.parent.feature_id!=y.parent.feature_id||x.parent.parent_element_id!=y.parent.parent_element_id||
           x.parent.parent_face_id!=y.parent.parent_face_id||x.covered!=y.covered||
           output::Bits(x.projected_area)!=output::Bits(y.projected_area)||
           output::Bits(x.parent.half_thickness)!=output::Bits(y.parent.half_thickness)||
           output::Bits(x.area_enclosure.lower)!=output::Bits(y.area_enclosure.lower)||
           output::Bits(x.area_enclosure.upper)!=output::Bits(y.area_enclosure.upper))return false;
        for(unsigned n=0;n<4;++n) {
            if(x.parent.nodes[n]!=y.parent.nodes[n])return false;
            const auto& p=x.reference_projection[n];const auto& q=y.reference_projection[n];
            if(output::Bits(p.x)!=output::Bits(q.x)||output::Bits(p.y)!=output::Bits(q.y)||output::Bits(p.z)!=output::Bits(q.z))return false;
        }
    }
    return true;
}
constexpr double kFailedRatio=std::numeric_limits<double>::max();
bool DifferenceUpper(double a,double b,double& upper) {
    GuidedStudyInterval difference;
    if(!sd::bounds::Difference(a,b,&difference))return false;
    upper=std::max(std::abs(difference.lower),std::abs(difference.upper));return true;
}
double BudgetLower(double fine,double absolute) {
    // Five percent is the exact ratio 1/20. Lower the denominator and raise
    // the numerator so rounded diagnostics cannot turn a failed gate into 1.
    const double magnitude=std::abs(fine);
    if(magnitude<=absolute)return absolute;
    GuidedStudyInterval relative;
    if(sd::bounds::DividePositive({magnitude,magnitude},20,&relative))return std::max(absolute,relative.lower);
    // Positive division underflow is harmless here: the absolute floor is
    // at least denorm_min and therefore exceeds this relative contribution.
    return magnitude/20==0?absolute:0;
}
double QuotientUpper(double numerator,double denominator) {
    if(!std::isfinite(denominator)||denominator<=0)return kFailedRatio;
    if(numerator==0)return 0;
    if(numerator==denominator)return 1;
    GuidedStudyInterval quotient;
    if(sd::bounds::DividePositive({numerator,numerator},denominator,&quotient))
        return numerator<denominator?std::min(1.,quotient.upper):quotient.upper;
    if(numerator/denominator==0)return std::numeric_limits<double>::denorm_min();
    // Overflow has a truthful finite fail marker; no infinity enters JSON.
    return kFailedRatio;
}
double Ratio(double coarse,double fine,double absolute,double error_a=0,double error_b=0) {
    double difference=0,uncertainty=0,numerator=0;
    if(!DifferenceUpper(coarse,fine,difference)||
       !sd::bounds::AddScalar(error_a,error_b,true,&uncertainty)||
       !sd::bounds::AddScalar(difference,uncertainty,true,&numerator))return kFailedRatio;
    return QuotientUpper(numerator,BudgetLower(fine,absolute));
}
double EventRatio(const GuidedStudyEvent& a,const GuidedStudyEvent& b,double coarse_h) {
    double floor=0,coarse_width=0,fine_width=0,lower_difference=0,upper_difference=0;
    if(!sd::bounds::MultiplyScalar(2,coarse_h,false,&floor)||
       !DifferenceUpper(a.upper_time,a.lower_time,coarse_width)||
       !DifferenceUpper(b.upper_time,b.lower_time,fine_width)||
       !DifferenceUpper(a.lower_time,b.lower_time,lower_difference)||
       !DifferenceUpper(a.upper_time,b.upper_time,upper_difference))return kFailedRatio;
    return QuotientUpper(std::max({coarse_width,fine_width,lower_difference,upper_difference}),BudgetLower(b.upper_time,floor));
}
bool Evidence(const GuidedStudySummary& s) {
    // Reject roundoff-only changes, without adding a new physical amplitude
    // requirement: these floors are far below the frozen response comparisons.
    constexpr double angular_roundoff=128*std::numeric_limits<double>::epsilon();
    const double curvature_roundoff=angular_roundoff*std::max(1.,s.maximum_abs_curvature)/(.1*.1);
    return s.certified_partial_area&&s.certified_unequal_nodal_force&&s.normal_wall_impulse.lower>0&&s.separated_rebounding&&
        s.maximum_abs_curvature>curvature_roundoff&&s.maximum_curvature-s.minimum_curvature>curvature_roundoff&&
        (s.maximum_rotation[0]-s.minimum_rotation[0]>angular_roundoff||s.maximum_rotation[1]-s.minimum_rotation[1]>angular_roundoff);
}
// Inputs have already passed their separate refinement or wall-variant
// admission. The second response supplies every relative comparison scale.
bool CompareResponses(const GuidedStudyData& a,const GuidedStudyData& b,
                      GuidedStudyComparison& output,std::string& error,const char* passed,const char* failed) {
    GuidedStudyComparison next;
    const double energy_floor=1e-6*b.config.initial_energy;
    if(!std::isfinite(energy_floor)||energy_floor<=0)return sd::Reject(error,"Study energy comparison floor is unrepresentable");
    for(std::size_t i=0;i<kGuidedStudySamples;++i) {
        const auto& x=a.samples[i];const auto& y=b.samples[i];
        if(!sd::TimeMatches(x.time,y.time,a.config.fixed_dt,std::max(x.epoch,y.epoch)))
            return sd::Reject(error,"Study common physical sample times do not match");
        for(unsigned j=0;j<2;++j) {
            next.displacement_ratio=std::max(next.displacement_ratio,Ratio(x.normal_displacement[j],y.normal_displacement[j],1e-6));
            next.velocity_ratio=std::max(next.velocity_ratio,Ratio(x.normal_velocity[j],y.normal_velocity[j],1e-5));
            next.rotation_ratio=std::max(next.rotation_ratio,Ratio(x.world_z_rotation[j],y.world_z_rotation[j],1e-5));
        }
        next.force_ratio=std::max(next.force_ratio,Ratio(x.normal_wall_force.value,y.normal_wall_force.value,1e-5,
            x.normal_wall_force.error,y.normal_wall_force.error));
        for(std::size_t j=0;j<x.energy.size();++j)
            next.energy_ratio=std::max(next.energy_ratio,Ratio(x.energy[j],y.energy[j],energy_floor,
                j==4?x.contact_potential.error:0,j==4?y.contact_potential.error:0));
        next.energy_ratio=std::max(next.energy_ratio,Ratio(x.shell_bending_energy,y.shell_bending_energy,energy_floor));
    }
    const auto& x=a.summary;const auto& y=b.summary;
    next.force_ratio=std::max(next.force_ratio,Ratio(x.sampled_peak_normal_force.value,y.sampled_peak_normal_force.value,1e-5,
        x.sampled_peak_normal_force.error,y.sampled_peak_normal_force.error));
    next.impulse_ratio=Ratio(x.normal_wall_impulse.value,y.normal_wall_impulse.value,2e-6,x.normal_wall_impulse.error,y.normal_wall_impulse.error);
    next.penetration_ratio=Ratio(x.maximum_penetration,y.maximum_penetration,1e-6);
    next.events_complete=x.activation.observed&&y.activation.observed&&x.pressure_release.observed&&y.pressure_release.observed;
    if(next.events_complete)next.event_ratio=std::max(EventRatio(x.activation,y.activation,a.config.fixed_dt),
                                                    EventRatio(x.pressure_release,y.pressure_release,a.config.fixed_dt));
    next.energy_envelopes=x.maximum_certified_energy_relative_error<=.01&&y.maximum_certified_energy_relative_error<=.01;
    next.deforming_contact_evidence=Evidence(x)&&Evidence(y);
    next.passed=next.energy_envelopes&&next.deforming_contact_evidence&&next.events_complete&&
        std::max({next.displacement_ratio,next.velocity_ratio,next.rotation_ratio,next.force_ratio,next.impulse_ratio,
                  next.energy_ratio,next.event_ratio,next.penetration_ratio})<=1;
    next.diagnostic=next.passed?passed:failed;
    output=std::move(next);error.clear();return true;
}
} // namespace
bool ValidateGuidedPlateStudy(const GuidedStudyData& data,std::string& diagnostic) {
    if(!Valid(data))return sd::Reject(diagnostic,"Completed study record is malformed or incomplete");
    diagnostic.clear();return true;
}
bool CompareGuidedPlateStudies(const GuidedStudyData& a,const GuidedStudyData& b,
                              GuidedStudyComparison& output,std::string& error) {
    const auto& c=a.config;const auto& f=b.config;
    if(!Valid(a)||!Valid(b)||!SamePhysicalExperiment(a,b)||c.wall_binding_id!=f.wall_binding_id||
       c.integration_backend!=f.integration_backend||
       c.base_steps!=f.base_steps||2*c.refinement!=f.refinement||c.fixed_dt!=2*f.fixed_dt)
        return sd::Reject(error,"Study comparison inputs are incomplete, malformed or different experiments");
    return CompareResponses(a,b,output,error,"Frozen guided refinement comparisons pass",
        "Guided refinement failed a frozen response, certificate, event, energy or deformation/rebound gate");
}
bool CompareGuidedPlateWallStudies(const GuidedStudyData& derived,const GuidedStudyData& canonical,
                                  GuidedStudyComparison& output,std::string& error) {
    const auto& a=derived.config;const auto& b=canonical.config;
    if(!Valid(derived)||!Valid(canonical)||!SamePhysicalExperiment(derived,canonical)||
       a.integration_backend!=b.integration_backend||
       a.wall_binding_id==b.wall_binding_id||a.base_steps!=b.base_steps||a.refinement!=b.refinement||
       output::Bits(a.fixed_dt)!=output::Bits(b.fixed_dt)||
       output::Bits(a.total_reference_area.lower)!=output::Bits(b.total_reference_area.lower)||
       output::Bits(a.total_reference_area.upper)!=output::Bits(b.total_reference_area.upper))
        return sd::Reject(error,"Wall studies require distinct wall bindings and the same completed fixed-step physical experiment");
    return CompareResponses(derived,canonical,output,error,"Frozen guided wall-tessellation comparisons pass",
        "Guided wall tessellation failed a frozen response, certificate, event, energy or deformation/rebound gate");
}
} // namespace crash::case_data
