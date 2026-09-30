#pragma once
#include "Values.h"
#include "lib_src/elements/qeph/QephBatchFailureSection.h"
#include "lib_src/elements/t3/T3BatchFailureSection.h"

namespace resident_tab1_test {
template<class F> struct HostFamily {
  static constexpr bool Quad = std::is_same_v<F, placed::Q>;
  static constexpr auto Family = Quad ? fe::ShellBindingFamily::Qeph : fe::ShellBindingFamily::T3;
  using History = std::conditional_t<Quad, fe::qeph::History, fe::t3::History>;
  using Force = std::conditional_t<Quad, fe::qeph::ForceTrial, fe::t3::ForceTrial>;
  Source source;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  tl::util::HostArena mixed_arena, failure_arena;
  storage::MixedDeviceStorage* mixed = nullptr;
  storage::FailureDeviceStorage* failure = nullptr;
  std::array<History, Parents> accepted;
  std::array<Force, Parents> candidate;
  unsigned slab = 0;
  explicit HostFamily(Placement plane, unsigned mask = 0) : source(plane) {
    if (!source.Prepare(binding, catalog)) throw std::runtime_error("resident source");
    storage::MixedLayout ml;
    storage::FailureLayout fl;
    if (!ml.Initialize(Parents, 0, 1 << 20) || !fl.Initialize(Parents, 1 << 20) ||
        !mixed_arena.Initialize(ml.bytes) || !failure_arena.Initialize(fl.bytes)) {
      throw std::runtime_error("resident host layout");
    }
    mixed = ml.Construct(mixed_arena);
    failure = fl.Construct(failure_arena);
    if (!mixed || !failure) throw std::runtime_error("resident host construction");
    for (unsigned e = 0; e < Parents; ++e) {
      catalog.Law(Family, e, &mixed->law[e]);
      if (e == 0) catalog.ElasticParameters(Family, e, &mixed->elastic_parameters[e]);
      else catalog.Parameters(Family, e, &mixed->plastic.parameters[e]);
      const auto& row = source.failures[2 * e + (Quad ? 1 : 0)];
      failure->policy[e] = row.policy;
      failure->parameters[e] = row.constant;
      failure->tab1_parameters[e] = row.tab1;
      for (unsigned slot = 0; slot < 2; ++slot) {
        if (e == 2) failure->state[slot][e] = fe::ShellBatchFailureState::Constant();
        if (e == 3) {
          const auto history = tab1_test::Seed(mask);
          failure->state[slot][e] = storage::FailureState(history);
          mixed->plastic.section[slot][e].history = history.saved;
        }
      }
      if (InitializeHistory(Reference(e), {}, accepted[e]) != F::Status::kSuccess) {
        throw std::runtime_error("resident host history");
      }
    }
  }
  typename F::Reference Reference(unsigned parent) const {
    if constexpr (Quad) return binding.qeph_reference(parent);
    else return binding.t3_reference(parent);
  }
  auto Evaluate(unsigned parent, unsigned step) {
    const auto reference = Reference(parent);
    const auto interval = placed::Interval(reference, step);
    if constexpr (Quad) {
      return fe::qeph::batch_detail::EvaluateFailureSection(reference, accepted[parent], interval,
          *mixed, *failure, slab, parent, candidate[parent]);
    } else {
      return fe::t3::batch_detail::EvaluateFailureSection(reference, accepted[parent], interval,
          *mixed, *failure, slab, parent, candidate[parent]);
    }
  }
  void Accept() {
    for (unsigned e = 0; e < Parents; ++e) accepted[e] = candidate[e].proposed_history;
    slab = 1 - slab;
  }
};
} // namespace resident_tab1_test
