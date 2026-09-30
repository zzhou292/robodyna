#include "GuidedPlateStudyInternal.h"
#include <algorithm>
#include <limits>

namespace crash::case_data {
namespace sd=study_detail;
namespace {
bool Scalar(const GuidedStudyConfig& c,const GuidedPlateMetrics& m,GuidedStudySummary& s,std::string& error) {
    GuidedStudyCertificate force;if(!sd::Force(m.contact,force))return sd::Reject(error,"Study global force enclosure failed");
    const auto& previous=s.sampled_peak_normal_force;
    if(!sd::bounds::Certify(std::max(previous.value,force.value),
            {std::max(previous.lower,force.lower),std::max(previous.upper,force.upper)},&s.sampled_peak_normal_force))
        return sd::Reject(error,"Study sampled peak enclosure failed");
    double difference=0,sum=0;GuidedStudyInterval relative;
    if(!sd::bounds::AbsoluteDifferenceUpper(m.work.total_energy,c.initial_energy,&difference)||
       !sd::bounds::AddScalar(difference,m.contact.potential.error,true,&sum)||
       !sd::bounds::DividePositive({sum,sum},c.initial_energy,&relative))
        return sd::Reject(error,"Study certified energy error overflow");
    const double raw=std::abs(m.work.total_energy-c.initial_energy)/c.initial_energy;
    if(!std::isfinite(raw))return sd::Reject(error,"Study value energy error overflow");
    s.maximum_value_energy_relative_error=std::max(s.maximum_value_energy_relative_error,raw);
    s.maximum_certified_energy_relative_error=std::max(s.maximum_certified_energy_relative_error,relative.upper);
    s.maximum_penetration=std::max(s.maximum_penetration,m.contact.maximum_penetration);
    s.certified_partial_area=s.certified_partial_area||
        (m.contact.active_area.lower>0&&m.contact.active_area.upper<c.total_reference_area.lower);
    s.accepted_epoch=m.stamp.epoch;s.accepted_time=m.stamp.time;s.last_attempt=m.shell.attempt;return true;
}
void SampleSummary(const GuidedStudySample& sample,bool partial,bool unequal,GuidedStudySummary& s) {
    if(s.sample_count==0) {
        s.minimum_curvature=s.maximum_curvature=sample.curvature_proxy;
        s.minimum_rotation=s.maximum_rotation=sample.world_z_rotation;
    } else {
        s.minimum_curvature=std::min(s.minimum_curvature,sample.curvature_proxy);
        s.maximum_curvature=std::max(s.maximum_curvature,sample.curvature_proxy);
        for(unsigned i=0;i<2;++i) {
            s.minimum_rotation[i]=std::min(s.minimum_rotation[i],sample.world_z_rotation[i]);
            s.maximum_rotation[i]=std::max(s.maximum_rotation[i],sample.world_z_rotation[i]);
        }
    }
    s.maximum_abs_curvature=std::max(s.maximum_abs_curvature,std::abs(sample.curvature_proxy));
    s.certified_partial_area=s.certified_partial_area||partial;
    s.certified_unequal_nodal_force=s.certified_unequal_nodal_force||unequal;
    if(!s.separated_rebounding&&s.pressure_release.observed&&sample.epoch>=s.pressure_release.upper_epoch&&
       sample.contact_potential.upper==0&&sample.minimum_signed_gap.lower>0&&sample.tip_normal_velocity.upper<0) {
        s.separated_rebounding=true;s.separation_sample_epoch=sample.epoch;s.separation_sample_time=sample.time;
    }
    ++s.sample_count;
}
} // namespace

bool GuidedPlateStudy::Initialize(const GuidedStudyConfig& config,const GuidedPlateMetrics& initial,
                                 const GuidedPlateFrame& frame,std::string& error) {
    if(initialized_)return sd::Reject(error,"Study already initialized");
    if(!sd::ValidConfig(config)||initial.stamp.epoch||initial.stamp.time!=0||initial.contact.potential.upper!=0||
       !sd::Endpoint(config,initial,error))return sd::Reject(error,"Study initial endpoint/configuration is invalid");
    GuidedStudySample sample;bool partial=false,unequal=false;
    if(!sd::Sample(config,initial,frame,sample,partial,unequal,error))return false;
    try {
        GuidedStudyData next;next.config=config;next.initial_position=frame.position;next.initial_rotation=frame.rotation;
        if(!Scalar(config,initial,next.summary,error))return false;
        SampleSummary(sample,partial,unequal,next.summary);next.samples[0]=sample;
        data_=std::move(next);initialized_=true;error.clear();return true;
    } catch(const std::exception& e) {error=e.what();return false;}
}
bool GuidedPlateStudy::NeedsSample(std::uint64_t epoch) const noexcept {
    std::uint64_t expected=0;
    return initialized_&&!data_.complete&&GuidedStudySampleEpoch(data_.config,data_.summary.sample_count,expected)&&epoch==expected;
}
bool GuidedPlateStudy::Record(const GuidedPlateMetrics& m,const GuidedPlateFrame* frame,std::string& error) {
    if(!initialized_||data_.complete)return sd::Reject(error,"Study is not open for accepted records");
    const auto& c=data_.config;const auto& old=data_.summary;
    if(m.stamp.epoch!=old.accepted_epoch+1||m.stamp.time!=old.accepted_time+c.fixed_dt||m.shell.attempt<=old.last_attempt||
       bool(frame)!=NeedsSample(m.stamp.epoch))return sd::Reject(error,"Study endpoint is skipped/stale or lacks its scheduled frame");
    if(!sd::Endpoint(c,m,error))return false;
    GuidedStudySummary next=old;
    GuidedStudyCertificate applied;GuidedStudyInterval increment,truth;
    if(!sd::Force(m.applied_contact,applied)||
       m.wall_impulse.x!=old.normal_wall_impulse.value+c.fixed_dt*m.applied_contact.wall_reaction.x||
       !sd::bounds::Scale({applied.lower,applied.upper},c.fixed_dt,&increment)||
       !sd::bounds::Add({old.normal_wall_impulse.lower,old.normal_wall_impulse.upper},increment,&truth)||
       !sd::bounds::Certify(m.wall_impulse.x,truth,&next.normal_wall_impulse))
        return sd::Reject(error,"Study base-force impulse or its accumulation enclosure is invalid");
    if(!Scalar(c,m,next,error))return false;
    auto inactive_epoch=last_inactive_epoch_,active_epoch=last_active_epoch_;
    double inactive_time=last_inactive_time_,active_time=last_active_time_;
    const bool inactive=m.contact.potential.upper==0,active=m.contact.potential.lower>0;
    if(!next.activation.observed) {
        if(inactive) {inactive_epoch=m.stamp.epoch;inactive_time=m.stamp.time;}
        else if(active) {
            next.activation={true,inactive_epoch,m.stamp.epoch,inactive_time,m.stamp.time};
            active_epoch=m.stamp.epoch;active_time=m.stamp.time;
        }
    } else {
        if(active) {active_epoch=m.stamp.epoch;active_time=m.stamp.time;}
        if(inactive&&!next.pressure_release.observed)
            next.pressure_release={true,active_epoch,m.stamp.epoch,active_time,m.stamp.time};
    }
    GuidedStudySample sample;bool partial=false,unequal=false;
    if(frame) {
        if(!sd::Sample(c,m,*frame,sample,partial,unequal,error))return false;
        SampleSummary(sample,partial,unequal,next);
    }
    // No fallible work after this point. Keep the fixed history in place.
    if(frame)data_.samples[old.sample_count]=sample;
    data_.summary=next;last_inactive_epoch_=inactive_epoch;last_inactive_time_=inactive_time;
    last_active_epoch_=active_epoch;last_active_time_=active_time;error.clear();return true;
}
bool GuidedPlateStudy::Finish(GuidedStudyData& output,std::string& error) {
    if(!initialized_||data_.summary.accepted_epoch!=data_.config.base_steps*data_.config.refinement||
       data_.summary.sample_count!=kGuidedStudySamples||
       !sd::TimeMatches(data_.summary.accepted_time,data_.config.horizon,data_.config.fixed_dt,data_.summary.accepted_epoch))
        return sd::Reject(error,"Study horizon/common sample schedule is incomplete");
    try { auto next=data_;next.complete=true;output=std::move(next);data_.complete=true;error.clear();return true; }
    catch(const std::exception& e) {error=e.what();return false;}
}
static_assert(sizeof(GuidedStudyData)<256*1024,"Revisit bounded study host storage");
} // namespace crash::case_data
