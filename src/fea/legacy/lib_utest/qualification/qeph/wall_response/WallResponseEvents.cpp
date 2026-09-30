#include "WallResponseInternal.h"

namespace tl::qualification::qeph::wall_response {
namespace d=detail;
namespace b=contact::q4_bounds;
namespace {
bool Domain(const Sample& s) {
  return s.relative_displacement/free_response::Side<=1e-3&&s.rotation_angle<=1e-3&&s.strain<=1e-3&&s.thickness_curvature<=1e-3&&
    s.minimum_area_ratio>=.999&&s.maximum_area_ratio<=1.001&&s.minimum_thickness_ratio>=.999&&s.maximum_thickness_ratio<=1.001;
}
EventBracket Bracket(const Summary& previous,const Sample& sample) {
  return {true,previous.last_epoch,sample.epoch,previous.last_time,sample.time};
}
bool EventTruth(const Model& m,const EventBracket& bracket,unsigned event,Interval& out) {
  const long double entry=wr::InitialGap/static_cast<long double>(wr::ImpactSpeed),pi=std::acos(-1.L);
  long double lo=entry,hi=entry;
  if(event) {
    lo=INFINITY; hi=0;
    for(unsigned n=0;n<m.fields().nodes;++n) {
      const auto& r=m.screened().mass_rates()[n];
      lo=std::min(lo,entry+(event==1?.5L:1.L)*pi/std::sqrt(static_cast<long double>(r.upper)));
      hi=std::max(hi,entry+(event==1?.5L:1.L)*pi/std::sqrt(static_cast<long double>(r.lower)));
    }
  }
  const double allowance=d::Roundoff*(Horizon+m.time_scale()); Interval truth;
  if(!b::Add({static_cast<double>(lo),static_cast<double>(hi)},{-allowance,allowance},&truth))return false;
  return d::Difference({bracket.before_time,bracket.after_time},truth,m.time_scale(),out);
}
bool UpdateEvent(const Model& m,const Config& config,const EventBracket& event,unsigned kind,Summary& result,std::string& error) {
  if(kind!=1) {
    wr::WallSwitchingSchedule schedule;
    if(!wr::BuildWallSwitchingSchedule(m.screened(),Step(config),schedule,error))return false;
    const auto expected=kind==0?schedule.entry_base_epoch:schedule.exit_base_epoch;
    if(event.after_epoch+1<expected||event.after_epoch>expected+1)return d::Fail(error,"Actual contact event is outside screened epoch window");
  }
  Interval difference;
  if(!EventTruth(m,event,kind,difference))return d::Fail(error,"Contact event analytic comparison overflow");
  if(difference.upper>result.maximum_analytic_difference.upper)result.analytic_time=event.after_time;
  d::Maximum(result.maximum_analytic_difference,difference); return true;
}
}
bool StageSummary(const Model& m,const Config& config,const Summary& previous,const Sample& sample,Summary& output,std::string& error) {
  if(!ValidateSample(m,config,sample,error))return false;
  if(!Domain(sample))return d::Fail(error,"Wall response deformation domain rejected");
  Summary result=previous; const unsigned full=(1u<<m.fields().nodes)-1;
  if(sample.mask!=0&&sample.mask!=full)return d::Fail(error,"Broadside response has a mixed active mask");
  if(!previous.initialized) {
    if(sample.epoch!=0||sample.mask||sample.maximum_gap>=0)return d::Fail(error,"Response must begin separated at physical epoch zero");
    result=Summary{}; result.initialized=true;
  } else {
    if(previous.last_epoch>=Steps(config)||sample.epoch!=previous.last_epoch+1||previous.last_time!=previous.last_epoch*Step(config))
      return d::Fail(error,"Nonconsecutive response endpoint");
    switch(previous.phase) {
      case Phase::BeforeEntry:
        if(sample.mask==full) {
          if(sample.minimum_gap<0||sample.minimum_velocity<=0)return d::Fail(error,"Entry lacks incoming full contact");
          result.entry=Bracket(previous,sample); result.phase=Phase::Compression;
          if(!UpdateEvent(m,config,result.entry,0,result,error))return false;
        }
        break;
      case Phase::Compression:
        if(!sample.mask)return d::Fail(error,"Release occurred without a resolved velocity reversal");
        if(sample.maximum_velocity<0) {
          result.peak=Bracket(previous,sample); result.phase=Phase::Unloading;
          if(!UpdateEvent(m,config,result.peak,1,result,error))return false;
        }
        break;
      case Phase::Unloading:
        if(!sample.mask) {
          if(sample.maximum_gap>=0||sample.maximum_velocity>=0)return d::Fail(error,"Zero force does not establish outgoing separation");
          result.exit=Bracket(previous,sample); result.phase=Phase::Separated;
          if(!UpdateEvent(m,config,result.exit,2,result,error))return false;
        }
        break;
      case Phase::Separated:
        if(sample.mask||sample.maximum_gap>=0||sample.maximum_velocity>=0)return d::Fail(error,"Reentry or nonoutgoing separated state");
        break;
    }
  }
  result.last_epoch=sample.epoch; result.last_time=sample.time; result.last_mask=sample.mask;
  result.last_minimum_velocity=sample.minimum_velocity; result.last_maximum_velocity=sample.maximum_velocity;
  if(sample.absolute_residual.upper>result.maximum_absolute_residual.upper)result.residual_time=sample.time;
  d::Maximum(result.maximum_absolute_residual,sample.absolute_residual);
  if(sample.analytic_difference.upper>result.maximum_analytic_difference.upper)result.analytic_time=sample.time;
  d::Maximum(result.maximum_analytic_difference,sample.analytic_difference);
  if(sample.maximum_gap>result.maximum_depth) {result.maximum_depth=sample.maximum_gap;result.depth_time=sample.time;}
  if(sample.contact.resultant.value>result.maximum_resultant) {
    result.maximum_resultant=sample.contact.resultant.value;result.resultant_time=sample.time;
  }
  // Independent maxima enclose the true maximum even when certificates at
  // neighboring near-peak samples have different widths. Time names max value.
  result.maximum_resultant_error=std::max(result.maximum_resultant_error,sample.contact.resultant.error);
  result.maximum_relative_displacement=std::max(result.maximum_relative_displacement,sample.relative_displacement);
  result.maximum_rotation=std::max(result.maximum_rotation,sample.rotation_angle);result.maximum_strain=std::max(result.maximum_strain,sample.strain);
  result.maximum_thickness_curvature=std::max(result.maximum_thickness_curvature,sample.thickness_curvature);
  result.minimum_area_ratio=std::min(result.minimum_area_ratio,sample.minimum_area_ratio);result.maximum_area_ratio=std::max(result.maximum_area_ratio,sample.maximum_area_ratio);
  result.minimum_thickness_ratio=std::min(result.minimum_thickness_ratio,sample.minimum_thickness_ratio);result.maximum_thickness_ratio=std::max(result.maximum_thickness_ratio,sample.maximum_thickness_ratio);
  double energy_bound=0;
  if(!b::MultiplyScalar(MaximumEnergyRatio,m.energy(),false,&energy_bound)||result.maximum_absolute_residual.upper>energy_bound||
     result.maximum_analytic_difference.upper>MaximumAnalyticError)return d::Fail(error,"Wall response energy/analytic gate rejected before publication");
  output=result; error.clear(); return true;
}
bool CompleteSummary(const Model& m,const Config& config,const Summary& s,std::string& error) {
  if(!m.prepared()||!ValidConfig(config)||config.cells!=m.fields().cells||!s.initialized||s.last_epoch!=Steps(config)||s.last_time!=Horizon||
     s.phase!=Phase::Separated||s.last_mask||!s.entry.observed||!s.peak.observed||!s.exit.observed||
     !(s.entry.after_epoch<s.peak.after_epoch&&s.peak.after_epoch<s.exit.after_epoch)||s.last_maximum_velocity>=0||
     !(s.maximum_depth>0&&s.maximum_depth<=wr::MaximumDepth)||!(s.maximum_resultant>s.maximum_resultant_error))
    return d::Fail(error,"Full incoming/compression/rebound horizon is incomplete");
  for(double x:{s.residual_time,s.analytic_time,s.maximum_depth,s.maximum_resultant,s.maximum_resultant_error,s.depth_time,s.resultant_time,
      s.maximum_relative_displacement,s.maximum_rotation,s.maximum_strain,s.maximum_thickness_curvature,s.minimum_area_ratio,s.maximum_area_ratio,
      s.minimum_thickness_ratio,s.maximum_thickness_ratio})if(!std::isfinite(x)||x<0)return d::Fail(error,"Invalid full-response summary scalar");
  if(s.residual_time>Horizon||s.analytic_time>Horizon||s.depth_time>Horizon||s.resultant_time>Horizon||
     s.maximum_relative_displacement/free_response::Side>1e-3||s.maximum_rotation>1e-3||s.maximum_strain>1e-3||s.maximum_thickness_curvature>1e-3||
     s.minimum_area_ratio<.999||s.maximum_area_ratio>1.001||s.minimum_thickness_ratio<.999||s.maximum_thickness_ratio>1.001)
    return d::Fail(error,"Full response deformation/observation domain rejected");
  for(const auto& event:{s.entry,s.peak,s.exit})if(event.after_epoch!=event.before_epoch+1||event.after_epoch>Steps(config)||
      event.before_time!=event.before_epoch*Step(config)||event.after_time!=event.after_epoch*Step(config))return d::Fail(error,"Invalid retained event bracket");
  Summary events;
  if(!UpdateEvent(m,config,s.entry,0,events,error)||!UpdateEvent(m,config,s.peak,1,events,error)||!UpdateEvent(m,config,s.exit,2,events,error))return false;
  if(s.maximum_analytic_difference.lower<events.maximum_analytic_difference.lower||s.maximum_analytic_difference.upper<events.maximum_analytic_difference.upper)
    return d::Fail(error,"Analytic summary omits a retained event comparison");
  double energy_bound=0;
  if(!d::Valid(s.maximum_absolute_residual)||s.maximum_absolute_residual.lower<0||!d::Valid(s.maximum_analytic_difference)||s.maximum_analytic_difference.lower<0||
     !b::MultiplyScalar(MaximumEnergyRatio,m.energy(),false,&energy_bound)||s.maximum_absolute_residual.upper>energy_bound||s.maximum_analytic_difference.upper>MaximumAnalyticError)
    return d::Fail(error,"Completed response has a failed numerical summary");
  Interval depth,force;
  if(!PeakTruth(m,s,depth,force,error))return false;
  if(depth.upper>MaximumAnalyticError||force.upper>MaximumAnalyticError)return d::Fail(error,"Continuous peak depth/resultant truth rejected");
  error.clear(); return true;
}
bool PeakTruth(const Model& m,const Summary& s,Interval& depth_difference,Interval& force_difference,std::string& error) {
  if(!m.prepared()||!std::isfinite(s.maximum_depth)||s.maximum_depth<0||!std::isfinite(s.maximum_resultant)||s.maximum_resultant<0||
     !std::isfinite(s.maximum_resultant_error)||s.maximum_resultant_error<0)return d::Fail(error,"Invalid continuous peak inputs");
  long double low_depth=0,high_depth=0,force_upper=0,force_at_common_peak=0,force_allowance=0;
  const long double speed=wr::ImpactSpeed,pi=std::acos(-1.L),time=pi/(2*m.screened().maximum_frequency());
  for(unsigned n=0;n<m.fields().nodes;++n) {
    const auto& r=m.screened().mass_rates()[n];
    const long double lo=std::sqrt(static_cast<long double>(r.lower)),hi=std::sqrt(static_cast<long double>(r.upper));
    const long double omega=std::sqrt(static_cast<long double>(r.value)),mass=m.fields().mass[n];
    low_depth=std::max(low_depth,speed/hi);high_depth=std::max(high_depth,speed/lo);
    force_upper+=mass*speed*hi;
    force_at_common_peak+=mass*speed*omega*std::sin(omega*time);
    const long double rate_error=std::max(static_cast<long double>(r.value)-r.lower,static_cast<long double>(r.upper)-r.value);
    force_allowance+=mass*speed/(2*lo)*(1+hi*time)*rate_error;
  }
  // Lower: an actual common time; upper: sum of individual node maxima.
  // This avoids assuming all certified mass-rate values are exactly equal.
  const double depth_allowance=d::Roundoff*(wr::TargetDepth+static_cast<double>(high_depth));
  const double force_roundoff=d::Roundoff*(m.force_scale()+static_cast<double>(force_upper));
  Interval expected_depth,expected_force,actual_force,depth,force;
  if(!b::Add({static_cast<double>(low_depth),static_cast<double>(high_depth)},{-depth_allowance,depth_allowance},&expected_depth)||
     !b::Add({static_cast<double>(force_at_common_peak-force_allowance),static_cast<double>(force_upper)},
             {-force_roundoff,force_roundoff},&expected_force)||
     !d::Enclose(s.maximum_resultant,s.maximum_resultant_error,actual_force)||
     !d::Difference({s.maximum_depth,s.maximum_depth},expected_depth,wr::TargetDepth,depth)||
     !d::Difference(actual_force,expected_force,m.force_scale(),force))return d::Fail(error,"Continuous peak comparison overflow");
  depth_difference=depth;force_difference=force;error.clear();return true;
}
} // namespace tl::qualification::qeph::wall_response
