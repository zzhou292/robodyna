#include "WallRawReadFields.h"
#include <utility>

namespace tl::qualification::qeph::wall_recurrence::read_detail {
MovingMatrixProbe Native(const io::Value& v,unsigned cells,double h,double boost,unsigned amplitude) {
  MovingMatrixProbe out; out.velocity=Vec<Vec3>(Field(v,"physical_velocity_baseline_m_s"));
  out.baseline_complete=Boolean(Field(v,"baseline_complete"));
  out.baseline=Vector(Field(v,"normalized_baseline"),!out.baseline_complete);
  auto& p=out.derivative; p.amplitude=Number(Field(v,"amplitude"));
  p.complete=Boolean(Field(v,"matrix_complete")); p.completed_columns=Count(Field(v,"completed_columns"),194);
  p.diagnostic=Text(Field(v,"diagnostic")); p.full=Matrix(Field(v,"full_matrix"),!p.complete);
  io::Require(Same(raw_detail::DescribeNative(out,cells,h,boost,amplitude),v),"Native probe fields/identity fail owning round trip");
  return out;
}
ContactBranchProbe Branch(const io::Value& v,unsigned cells,double h,double boost,unsigned branch) {
  ContactBranchProbe out; out.h=Number(Field(v,"fixed_dt_s")); out.normal_velocity=Number(Field(v,"normal_velocity_m_s"));
  const auto kind=Text(Field(v,"branch")); io::Require(kind=="inactive"||kind=="active","Unknown raw contact branch");
  out.branch=kind=="inactive"?ContactBranch::Inactive:ContactBranch::Active;
  out.complete=Boolean(Field(v,"local_derivative_checks_complete")); out.passed=Boolean(Field(v,"local_derivative_checks_passed"));
  out.full=Matrix(Field(v,"full_operator"),!out.complete); out.baseline_complete=Boolean(Field(v,"baseline_complete"));
  out.baseline=Sample(Field(v,"baseline"),cells,out.baseline_complete); out.diagnostic=Text(Field(v,"diagnostic"));
  const auto& directions=Field(v,"directions"); io::Require(directions.IsArray()&&directions.Size()<=2*(cells+1)+7,"Unbounded raw directions");
  for(const auto& value:directions.GetArray()) {
    ContactDirectionProbe d; d.direction.name=Text(Field(value,"name")); d.direction.value=Vector(Field(value,"direction"));
    d.completed_samples=Count(Field(value,"completed_samples"),3); d.complete=Boolean(Field(value,"checks_complete"));
    d.passed=Boolean(Field(value,"checks_passed")); d.diagnostic=Text(Field(value,"diagnostic"));
    const auto& samples=Field(value,"samples"); const auto& quotients=Field(value,"one_sided_quotients");
    const auto& residuals=Field(value,"residual_upper"); const auto& budgets=Field(value,"budget_lower");
    io::Require(samples.IsArray()&&samples.Size()==3&&quotients.IsArray()&&quotients.Size()==2&&
      residuals.IsArray()&&residuals.Size()==2&&budgets.IsArray()&&budgets.Size()==2,"Malformed raw direction arrays");
    for(unsigned i=0;i<3;++i) d.samples[i]=Sample(samples[i],cells,i<d.completed_samples);
    for(unsigned i=0;i<2;++i) {
      d.quotients[i]=Vector(quotients[i],!d.complete); d.residual_upper[i]=Scalar(residuals[i],!d.complete);
      d.budget[i]=Scalar(budgets[i],!d.complete);
    }
    out.directions.push_back(std::move(d));
  }
  io::Require(Same(raw_detail::DescribeBranch(out,cells,h,boost,branch),v),"Contact probe fields/identity fail owning round trip");
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence::read_detail
