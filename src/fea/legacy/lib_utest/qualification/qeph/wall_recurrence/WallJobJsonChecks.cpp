#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence::job_json {
io::Document Contact(const WallContactRecheck& a,unsigned dimension,unsigned nodes) {
  io::Require((a.branch==ContactBranch::Inactive||a.branch==ContactBranch::Active)&&a.directions.size()<=nodes+7&&
    a.completed_directions<=a.directions.size(),"Invalid recomputed contact shape/identity");
  io::Require(!a.complete||(a.input_valid&&a.operator_difference.complete&&a.baseline.complete&&a.directions.size()==nodes+7&&
    a.completed_directions==nodes+7),"Completed contact recheck lacks component evidence");
  io::Require(!a.passed||(a.operator_difference.passed&&a.baseline.passed),"Passed contact recheck lacks baseline/operator evidence");
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  io::String(d,"branch",a.branch==ContactBranch::Inactive?"inactive":"active"); io::Boolean(d,"input_valid",a.input_valid);
  io::Integer(d,"completed_directions",a.completed_directions);
  raw_detail::Field(d,"operator_difference",Difference(a.operator_difference)); raw_detail::Field(d,"baseline",Baseline(a.baseline,dimension));
  io::Value directions(rapidjson::kArrayType);
  for(const auto& p:a.directions) {
    io::Require(!a.complete||p.complete,"Completed contact recheck lacks direction evidence");
    io::Require(!a.passed||p.passed,"Passed contact recheck lacks passing direction evidence");
    io::Require(p.name.size()<=256,"Unbounded recomputed direction name");
    io::Document direction; direction.SetObject(); Status(direction,p.complete,p.passed,p.diagnostic);
    io::String(direction,"name",p.name); io::Boolean(direction,"samples_valid",p.samples_valid);
    io::Value quotients(rapidjson::kArrayType),residuals(rapidjson::kArrayType),budgets(rapidjson::kArrayType);
    for(unsigned i=0;i<2;++i) {
      io::Require((p.quotients[i].size()==0||p.quotients[i].size()==dimension)&&(!p.complete||p.quotients[i].size()==dimension),
        "Invalid recomputed quotient dimensions");
      quotients.PushBack(raw_detail::Vector(direction,p.quotients[i],!p.complete),direction.GetAllocator());
      residuals.PushBack(raw_detail::Scalar(direction,p.residual_upper[i],!p.complete),direction.GetAllocator());
      budgets.PushBack(raw_detail::Scalar(direction,p.budget_lower[i],!p.complete),direction.GetAllocator());
    }
    direction.AddMember("quotients",quotients,direction.GetAllocator()); direction.AddMember("residual_upper",residuals,direction.GetAllocator());
    direction.AddMember("budget_lower",budgets,direction.GetAllocator()); raw_detail::Append(d,directions,direction);
  }
  d.AddMember("directions",directions,d.GetAllocator()); return d;
}
io::Document Step(const WallStepAnalysis& a) {
  io::Require(FrozenStep(a.h),"Invalid derived step identity"); io::Document d; d.SetObject();
  for(const auto& x:a.amplitudes) {
    io::Require(!a.complete||x.complete,"Completed step lacks amplitude evidence");
    io::Require(!a.passed||x.passed,"Passed step lacks passing amplitude evidence");
  }
  for(const auto& x:a.contact) {
    io::Require(!a.complete||x.complete,"Completed step lacks contact evidence");
    io::Require(!a.passed||x.passed,"Passed step lacks passing contact evidence");
  }
  for(unsigned i=0;i<2;++i) {
    io::Require(!a.complete||(a.native_amplitude_comparisons[i].complete&&a.derived_amplitude_comparisons[i].complete),
      "Completed step lacks comparison evidence");
    io::Require(!a.passed||(a.native_amplitude_comparisons[i].passed&&a.derived_amplitude_comparisons[i].passed),
      "Passed step lacks passing comparison evidence");
  }
  io::Number(d,"fixed_dt_s",a.h); Status(d,a.complete,a.passed,a.diagnostic);
  io::Value native(rapidjson::kArrayType),derived(rapidjson::kArrayType);
  for(const auto& x:a.native_amplitude_comparisons) raw_detail::Append(d,native,Difference(x));
  for(const auto& x:a.derived_amplitude_comparisons) raw_detail::Append(d,derived,Comparison(x));
  d.AddMember("native_amplitude_comparisons",native,d.GetAllocator()); d.AddMember("derived_amplitude_comparisons",derived,d.GetAllocator()); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::job_json
