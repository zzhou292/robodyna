#include "WallRawJson.h"

namespace tl::qualification::qeph::wall_recurrence::raw_detail {
io::Document DescribeNative(const MovingMatrixProbe& probe,unsigned cells,double h,double velocity,unsigned amplitude) {
  const unsigned dimension=12*(2*(cells+1))+61*cells; const auto& p=probe.derivative;
  io::Require(amplitude<3&&FrozenStep(h)&&probe.velocity.x==velocity&&probe.velocity.y==0&&probe.velocity.z==0&&
    p.amplitude==recurrence::Amplitudes[amplitude]&&p.completed_columns<=dimension,"Changed raw native probe identity");
  io::Require((p.full.rows()==dimension&&p.full.cols()==dimension)||
    (!p.complete&&p.completed_columns==0&&p.full.size()==0),"Invalid raw native matrix dimensions");
  io::Require(p.completed_columns==0||p.full.leftCols(p.completed_columns).allFinite(),"Nonfinite completed native column");
  io::Require(!p.complete||(probe.baseline_complete&&p.completed_columns==dimension&&p.full.allFinite()),
              "Completed native probe has incomplete/nonfinite data");
  io::Require(!probe.baseline_complete||(probe.baseline.size()==dimension&&probe.baseline.allFinite()),"Invalid native baseline");
  io::Require(probe.baseline_complete||probe.baseline.size()==0,"Unpublished native baseline has data");
  io::Document d; d.SetObject(); io::Number(d,"fixed_dt_s",h); io::Number(d,"amplitude",p.amplitude);
  Vec(d,"physical_velocity_baseline_m_s",probe.velocity); io::Boolean(d,"baseline_complete",probe.baseline_complete);
  d.AddMember("normalized_baseline",Vector(d,probe.baseline,!probe.baseline_complete),d.GetAllocator());
  io::Boolean(d,"matrix_complete",p.complete); io::Integer(d,"completed_columns",p.completed_columns);
  io::String(d,"diagnostic",p.diagnostic); Field(d,"full_matrix",Matrix(p.full,!p.complete)); return d;
}
io::Document DescribeBranch(const ContactBranchProbe& p,unsigned cells,double h,double velocity,unsigned branch) {
  const unsigned nodes=2*(cells+1),dimension=12*nodes+61*cells;
  io::Require(branch<2&&p.h==h&&p.normal_velocity==velocity&&
    p.branch==(branch==0?ContactBranch::Inactive:ContactBranch::Active)&&p.directions.size()<=nodes+7,
    "Changed raw contact branch identity/capacity");
  io::Require((p.full.rows()==dimension&&p.full.cols()==dimension&&p.full.allFinite())||
    (!p.complete&&!p.baseline_complete&&p.directions.empty()&&p.full.size()==0),"Invalid partial contact branch operator");
  io::Require(!p.complete||(p.baseline_complete&&p.directions.size()==nodes+7),"Completed branch has missing samples");
  io::Require(!p.passed||p.complete,"Passed branch checks are incomplete");
  io::Document d; d.SetObject(); io::Number(d,"fixed_dt_s",h); io::Number(d,"normal_velocity_m_s",velocity);
  io::FiniteArray(d,"amplitudes",recurrence::Amplitudes.data(),3);
  io::String(d,"branch",branch==0?"inactive":"active"); Field(d,"full_operator",Matrix(p.full,!p.complete));
  io::Boolean(d,"baseline_complete",p.baseline_complete); Field(d,"baseline",DescribeContactSample(p.baseline,cells,p.baseline_complete));
  io::Boolean(d,"local_derivative_checks_complete",p.complete); io::Boolean(d,"local_derivative_checks_passed",p.passed);
  io::String(d,"diagnostic",p.diagnostic); io::Value directions(rapidjson::kArrayType);
  for(const auto& probe:p.directions) {
    io::Require(probe.direction.value.size()==dimension&&probe.completed_samples<=3&&
      (!probe.complete||probe.completed_samples==3)&&(!probe.passed||probe.complete),"Invalid directional raw record");
    io::Document q; q.SetObject(); io::String(q,"name",probe.direction.name);
    q.AddMember("direction",Vector(q,probe.direction.value),q.GetAllocator());
    io::Integer(q,"completed_samples",probe.completed_samples); io::Boolean(q,"checks_complete",probe.complete);
    io::Boolean(q,"checks_passed",probe.passed); io::String(q,"diagnostic",probe.diagnostic);
    io::Value samples(rapidjson::kArrayType),quotients(rapidjson::kArrayType);
    for(unsigned a=0;a<3;++a) Append(q,samples,DescribeContactSample(probe.samples[a],cells,a<probe.completed_samples));
    for(const auto& v:probe.quotients) {
      io::Require(v.size()==0||v.size()==dimension,"Invalid raw quotient dimension");
      io::Require(!probe.complete||v.size()==dimension,"Missing completed quotient");
      quotients.PushBack(Vector(q,v,!probe.complete),q.GetAllocator());
    }
    q.AddMember("samples",samples,q.GetAllocator()); q.AddMember("one_sided_quotients",quotients,q.GetAllocator());
    io::Value residuals(rapidjson::kArrayType),budgets(rapidjson::kArrayType);
    for(unsigned l=0;l<2;++l) {
      residuals.PushBack(Scalar(q,probe.residual_upper[l],!probe.complete),q.GetAllocator());
      budgets.PushBack(Scalar(q,probe.budget[l],!probe.complete),q.GetAllocator());
    }
    q.AddMember("residual_upper",residuals,q.GetAllocator()); q.AddMember("budget_lower",budgets,q.GetAllocator());
    Append(d,directions,q);
  }
  d.AddMember("directions",directions,d.GetAllocator()); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::raw_detail
