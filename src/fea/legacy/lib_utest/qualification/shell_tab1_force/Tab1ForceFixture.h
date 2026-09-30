#pragma once
#include "lib_src/elements/qeph/QephLayeredTab1.h"
#include "lib_src/elements/t3/T3LayeredTab1.h"
#include "../shell_failure_force/FailureForceFields.h"
#include "../shell_tab1_glass/Tab1Fixture.h"

namespace tab1_force_test {
namespace q=tl::fea::qeph;
namespace t=tl::fea::t3;
namespace sec=tl::fea::sections;
using tl::math::Vec3;
using failure_force_test::Interval;
using failure_force_test::Dt;
using failure_force_test::Append;
using failure_force_test::Exact;
using failure_force_test::ForceValues;

struct Q {
  using Status=q::Status;
  using Reference=q::ReferenceData;
  using History=q::LayeredTab1History;
  using Trial=q::LayeredTab1ForceTrial;
  static Reference ReferenceValue() {
    auto input=failure_force_test::Q::ReferenceValue().input;
    input.density=2500;
    input.young_modulus=70e9;
    input.poisson_ratio=.22;
    input.thickness=.003;
    Reference r;
    if(q::InitializeReference(input,r)!=Status::kSuccess) throw std::runtime_error("glass Q reference");
    return r;
  }
};
struct T {
  using Status=t::Status;
  using Reference=t::ReferenceData;
  using History=t::LayeredTab1History;
  using Trial=t::LayeredTab1ForceTrial;
  static Reference ReferenceValue() {
    auto input=failure_force_test::T::ReferenceValue().input;
    input.density=2500;
    input.young_modulus=70e9;
    input.poisson_ratio=.22;
    input.thickness=.003;
    Reference r;
    if(t::InitializeReference(input,r)!=Status::kSuccess) throw std::runtime_error("glass T reference");
    return r;
  }
};
template<class F> struct Fixture {
  sec::PointParameters material=tab1_test::Material();
  sec::ShellLayeredTab1Parameters failure{tab1_test::Table()};
  typename F::Reference reference=F::ReferenceValue();
  typename F::History accepted;
  explicit Fixture(unsigned mask=0) {
    if(InitializeLayeredTab1History(reference,material,failure,{},accepted)!=F::Status::kSuccess)
      throw std::runtime_error("glass family startup");
    accepted.section=tab1_test::Seed(mask);
  }
  auto Evaluate(unsigned step,typename F::Trial& output) const {
    return EvaluateLayeredTab1Force(reference,material,failure,accepted,
        Interval(reference,step),output);
  }
  void Accept(const typename F::Trial& trial) {
    accepted={trial.force.proposed_history,trial.section.history};
  }
};
inline std::vector<double> SectionValues(const sec::ShellLayeredTab1Result& r) {
  std::vector<double> values;
  Append(values,r.history.saved);
  for(const auto& p:r.history.failure) {
    Append(values,p.damage);
    Append(values,p.maximum_damage);
    Append(values,p.failure_time_s);
    Append(values,p.table_segment);
    Append(values,p.point_active);
  }
  for(const auto& p:r.history.current_force_point) Append(values,p.stress);
  Append(values,r.history.element_active);
  Append(values,r.current.history);
  Append(values,r.current.material_stress);
  Append(values,r.current.bending_stress);
  Append(values,r.current.reported_thickness);
  Append(values,r.current.diagnostics);
  Append(values,r.constitutive_increment);
  Append(values,r.caller_failure_increment);
  Append(values,r.removed_now);
  return values;
}
} // namespace tab1_force_test
