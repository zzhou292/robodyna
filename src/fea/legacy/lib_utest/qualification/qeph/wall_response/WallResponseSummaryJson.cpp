#include "WallResponseJson.h"

namespace tl::qualification::qeph::wall_response::json {
namespace {
constexpr NumberField<Summary> Numbers[]{
  {"last_time_s",&Summary::last_time},{"last_minimum_velocity_m_s",&Summary::last_minimum_velocity},
  {"last_maximum_velocity_m_s",&Summary::last_maximum_velocity},{"residual_extremum_time_s",&Summary::residual_time},
  {"analytic_extremum_time_s",&Summary::analytic_time},{"maximum_depth_m",&Summary::maximum_depth},
  {"maximum_resultant_N",&Summary::maximum_resultant},{"maximum_resultant_error_N",&Summary::maximum_resultant_error},
  {"depth_extremum_time_s",&Summary::depth_time},{"resultant_extremum_time_s",&Summary::resultant_time},
  {"maximum_relative_deformation_m",&Summary::maximum_relative_displacement},
  {"maximum_rotation_rad",&Summary::maximum_rotation},{"maximum_strain",&Summary::maximum_strain},
  {"maximum_thickness_curvature",&Summary::maximum_thickness_curvature},
  {"minimum_area_ratio",&Summary::minimum_area_ratio},{"maximum_area_ratio",&Summary::maximum_area_ratio},
  {"minimum_thickness_ratio",&Summary::minimum_thickness_ratio},{"maximum_thickness_ratio",&Summary::maximum_thickness_ratio}};
io::Document Event(const EventBracket& event) {
  io::Document d; d.SetObject(); io::Boolean(d,"observed",event.observed);
  io::Integer(d,"before_epoch",event.before_epoch); io::Integer(d,"after_epoch",event.after_epoch);
  io::Number(d,"before_time_s",event.before_time); io::Number(d,"after_time_s",event.after_time); return d;
}
EventBracket ReadEvent(const io::Value& d) {
  return {p::Boolean(d,"observed"),p::Integer(d,"before_epoch"),p::Integer(d,"after_epoch"),
    p::Number(d,"before_time_s"),p::Number(d,"after_time_s")};
}
}
io::Document SummaryReport(const Summary& s) {
  io::Document d; d.SetObject(); io::Boolean(d,"initialized",s.initialized);
  io::Integer(d,"last_epoch",s.last_epoch); io::Integer(d,"last_mask",s.last_mask);
  io::Integer(d,"phase",static_cast<unsigned>(s.phase)); WriteNumbers(d,s,Numbers);
  p::Object(d,"entry",Event(s.entry)); p::Object(d,"peak",Event(s.peak)); p::Object(d,"exit",Event(s.exit));
  IntervalField(d,"maximum_absolute_residual_J",s.maximum_absolute_residual);
  IntervalField(d,"maximum_normalized_analytic_difference",s.maximum_analytic_difference); return d;
}
Summary ReadSummary(const io::Value& d) {
  Summary s; s.initialized=p::Boolean(d,"initialized"); s.last_epoch=p::Integer(d,"last_epoch");
  s.last_mask=Count(d,"last_mask",(1u<<MaxNodes)-1); s.phase=static_cast<Phase>(Count(d,"phase",static_cast<unsigned>(Phase::Separated)));
  ReadNumbers(d,s,Numbers); s.entry=ReadEvent(p::Member(d,"entry"));
  s.peak=ReadEvent(p::Member(d,"peak")); s.exit=ReadEvent(p::Member(d,"exit"));
  s.maximum_absolute_residual=ReadInterval(d,"maximum_absolute_residual_J");
  s.maximum_analytic_difference=ReadInterval(d,"maximum_normalized_analytic_difference"); return s;
}
void RunMetadata(io::Document& d,const Run& r) {
  io::Integer(d,"owner_id",r.owner_id); io::Integer(d,"accepted_steps",r.accepted_steps);
  io::Integer(d,"attempted_steps",r.attempted_steps); io::Integer(d,"native_cell_intervals",r.native_cell_intervals);
  io::Boolean(d,"completed",r.completed); io::String(d,"failure",r.failure); io::Number(d,"elapsed_seconds",r.elapsed_seconds);
  Array(d,"ledger_maxima",r.ledger_maxima);
  io::Integer(d,"owned_owner_device_bytes",r.owner_device_bytes); io::Integer(d,"owned_batch_device_bytes",r.batch_device_bytes);
  io::Integer(d,"owned_wall_device_bytes",r.wall_device_bytes); io::Integer(d,"owner_allocations",r.owner_allocations);
  io::Integer(d,"batch_allocations",r.batch_allocations); io::Integer(d,"wall_allocations",r.wall_allocations);
}
void ReadRunMetadata(const io::Value& d,Run& r) {
  r.owner_id=p::Integer(d,"owner_id"); r.accepted_steps=p::Integer(d,"accepted_steps");
  r.attempted_steps=p::Integer(d,"attempted_steps"); r.native_cell_intervals=p::Integer(d,"native_cell_intervals");
  r.completed=p::Boolean(d,"completed"); r.failure=p::String(d,"failure"); r.elapsed_seconds=p::Number(d,"elapsed_seconds");
  ReadArray(d,"ledger_maxima",r.ledger_maxima);
  r.owner_device_bytes=p::Integer(d,"owned_owner_device_bytes"); r.batch_device_bytes=p::Integer(d,"owned_batch_device_bytes");
  r.wall_device_bytes=p::Integer(d,"owned_wall_device_bytes"); r.owner_allocations=p::Integer(d,"owner_allocations");
  r.batch_allocations=p::Integer(d,"batch_allocations"); r.wall_allocations=p::Integer(d,"wall_allocations");
}
} // namespace tl::qualification::qeph::wall_response::json
