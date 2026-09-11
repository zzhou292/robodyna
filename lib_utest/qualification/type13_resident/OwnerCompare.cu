// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type13_resident_test {
void Rig::CompareAssembly(const fe::NodalAssemblyView& view) {
  const auto n = source.domain.node_count();
  auto expected = loads;
  std::vector<double> magnitude(loads.size());
  std::vector<std::size_t> terms(n);
  for (std::size_t i = 0; i < loads.size(); ++i) {
    magnitude[i] = std::fabs(loads[i]);
  }
  for (std::size_t e = 0; e < native.size(); ++e) {
    for (unsigned local = 0; local < 2; ++local) {
      const auto node = source.contributions.records()[2*e+local].value.global_node;
      ++terms[node];
      const auto& value = native[e].endpoints[local];
      const double rhs[6] = {value.force_N.x, value.force_N.y, value.force_N.z,
                             value.couple_Nm.x, value.couple_Nm.y, value.couple_Nm.z};
      for (unsigned c = 0; c < 6; ++c) {
        expected[c*n+node] += rhs[c];
        magnitude[c*n+node] += std::fabs(rhs[c]);
      }
    }
  }
  const double* sources[] = {view.forces.force_x, view.forces.force_y, view.forces.force_z,
                             view.forces.couple_x, view.forces.couple_y, view.forces.couple_z};
  std::vector<double> actual(6*n);
  for (unsigned c = 0; c < 6; ++c) {
    ASSERT_EQ(cudaMemcpyAsync(actual.data()+c*n, sources[c], n*sizeof(double),
                               cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
  }
  ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
  for (std::size_t i = 0; i < actual.size(); ++i) {
    SCOPED_TRACE(i);
    // Native packet agreement plus ordered reduction roundoff, scaled only by
    // this node/component's incident terms. A small resultant after cancellation
    // does not erase the individual packet error budget or borrow another node's.
    const double relative = 2e-11 + (4 + 2*terms[i % n]) * std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(actual[i], expected[i], relative * std::max(1e-10, magnitude[i]));
  }
}

void Rig::ComparePrepared(const fe::NodalTrialToken& token,
                           const fe::NodalPreparedView& prepared,
                           const std::vector<t::Evaluation>& actual,
                           std::vector<t::Evaluation>& next) {
  Snapshot fields(source.domain.node_count());
  fe::NodalPreparedView copied;
  ASSERT_TRUE(Good(owner.CopyPrepared(token, fields.Buffer(), &copied)));
  ASSERT_TRUE(fe::trial_identity::SamePrepared(prepared, copied));
  next.resize(native.size());
  for (std::size_t e = 0; e < native.size(); ++e) {
    SCOPED_TRACE(e);
    const auto& connection = source.model.connections()[e];
    const auto& reference = source.model.startup(e)->reference;
    t::NativeEndpointKinematics nodes[2];
    for (unsigned local = 0; local < 2; ++local) {
      const auto n = source.contributions.records()[2*e+local].value.global_node;
      const auto read = [&](const std::vector<double>& v) {
        return t::Vec3{v[3*n], v[3*n+1], v[3*n+2]};
      };
      ASSERT_TRUE(detail::FromSI(source.model.units(), read(fields.x), read(fields.v), read(fields.w), nodes[local]));
    }
    next[e] = type13_recurrence_test::NativeEvaluate(*source.model.property(connection.property),
        reference, native[e].native_history, nodes, config.owner.fixed_dt/source.model.units().time_to_s);
    Agreement(actual[e], next[e]);
  }
}
} // namespace type13_resident_test
