#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence::job_json {
namespace {
io::Document Window(const WallSwitchingWindow& w) {
  io::Require(w.entry_shift>=-1&&w.entry_shift<=1&&w.exit_shift>=-1&&w.exit_shift<=1&&
    w.entry_base_epoch<=32768&&w.exit_base_epoch<=32768&&w.inactive_before<=32768&&w.active<=32768&&w.inactive_after<=32768,
    "Unbounded derived switching window");
  io::Document d; d.SetObject(); d.AddMember("entry_shift",w.entry_shift,d.GetAllocator()); d.AddMember("exit_shift",w.exit_shift,d.GetAllocator());
  io::Integer(d,"entry_base_epoch",w.entry_base_epoch); io::Integer(d,"exit_base_epoch",w.exit_base_epoch);
  io::Integer(d,"inactive_before",w.inactive_before); io::Integer(d,"active",w.active); io::Integer(d,"inactive_after",w.inactive_after); return d;
}
}
io::Document Context(const WallBranchAnalysis& a,unsigned dimension) {
  io::Require(FrozenStep(a.h)&&(a.metric.diagonal.size()==0||a.metric.diagonal.size()==dimension),"Invalid actual derived context identity");
  const auto& m=a.metric; const auto& s=a.schedule;
  io::Require((s.h==0||s.h==a.h)&&s.total_steps<=32768&&s.ordinary_steps<=32768&&s.scalar.size()<=32769&&
    s.entry_base_epoch<=32768&&s.exit_base_epoch<=32768,"Unbounded derived scalar schedule");
  io::Require(!s.complete||(s.h==a.h&&s.scalar.size()==s.total_steps+1&&s.ordinary_steps+1==s.total_steps),
    "Completed scalar schedule has inconsistent counts");
  io::Document d; d.SetObject(); io::Number(d,"fixed_dt_s",a.h); io::Integer(d,"dimension",dimension);
  io::Document metric; metric.SetObject(); const bool metric_partial=m.diagonal.size()==0;
  metric.AddMember("diagonal",raw_detail::Vector(metric,m.diagonal,metric_partial),metric.GetAllocator());
  Scalar(metric,"sound_speed_m_s",m.sound_speed,metric_partial); Scalar(metric,"eta",m.eta,metric_partial);
  Scalar(metric,"tangential_weight",m.tangential_weight,metric_partial); Scalar(metric,"normal_weight",m.normal_weight,metric_partial);
  Scalar(metric,"condition",m.condition,metric_partial);
  io::String(metric,"interpretation","Invertible fixed coordinate norm; no mass change, deleted mode or thermodynamic-energy claim");
  raw_detail::Field(d,"metric",metric);
  io::Document schedule; schedule.SetObject(); Status(schedule,s.complete,s.passed,s.diagnostic);
  Scalar(schedule,"fixed_dt_s",s.h,!s.complete); Scalar(schedule,"frequency_s_inverse",s.frequency,!s.complete);
  Scalar(schedule,"maximum_depth_m",s.maximum_depth,!s.complete);
  io::Integer(schedule,"total_steps",s.total_steps); io::Integer(schedule,"ordinary_steps",s.ordinary_steps);
  io::Integer(schedule,"entry_base_epoch",s.entry_base_epoch); io::Integer(schedule,"exit_base_epoch",s.exit_base_epoch);
  io::String(schedule,"scalar_row_fields","position_m, carried_velocity_m_s, active_at_base");
  io::Value states(rapidjson::kArrayType),windows(rapidjson::kArrayType);
  for(const auto& x:s.scalar) {
    io::Value row(rapidjson::kArrayType); row.PushBack(raw_detail::Scalar(schedule,x.position,!s.complete),schedule.GetAllocator());
    row.PushBack(raw_detail::Scalar(schedule,x.velocity,!s.complete),schedule.GetAllocator()); row.PushBack(x.active,schedule.GetAllocator());
    states.PushBack(row,schedule.GetAllocator());
  }
  for(const auto& w:s.windows) raw_detail::Append(schedule,windows,Window(w));
  schedule.AddMember("scalar_states",states,schedule.GetAllocator()); schedule.AddMember("windows",windows,schedule.GetAllocator());
  raw_detail::Field(d,"schedule",schedule); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::job_json
