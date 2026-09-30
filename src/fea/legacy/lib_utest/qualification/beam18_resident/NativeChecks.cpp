// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace beam18_resident_test {
void CompareResult(const b::Parent& parent, const b::Material& material,
    const b::Result& r, const beam18_force_test::NativeResult& native) {
  // Restore only the sealed identity wrapper for the already-qualified packet
  // comparator. No production force/material calculation supplies this oracle.
  b::ForceTrial value;
  b::ForceHistoryWriter::Store(parent.reference, material, r.history, r.stamp, value.proposed_history);
  value.geometry = r.geometry;
  value.rate = r.rate;
  std::copy_n(r.point, 4, value.point);
  std::copy_n(r.rhs_force_n, 2, value.rhs_force_n);
  std::copy_n(r.rhs_couple_nm, 2, value.rhs_couple_nm);
  value.diagnostics = r.diagnostics;
  beam18_force_test::Compare(value, native);
}
void SameResults(const Results& a, const Results& bvalues) {
  ASSERT_EQ(a.size(), bvalues.size());
  for (std::size_t p = 0; p < a.size(); ++p) {
    SCOPED_TRACE(p);
    auto packet = [](const b::Result& r) {
      b::ForceTrial v;
      b::ForceHistoryWriter::Store({}, {}, r.history, r.stamp, v.proposed_history);
      v.geometry = r.geometry; v.rate = r.rate; v.diagnostics = r.diagnostics;
      std::copy_n(r.point, 4, v.point);
      std::copy_n(r.rhs_force_n, 2, v.rhs_force_n);
      std::copy_n(r.rhs_couple_nm, 2, v.rhs_couple_nm);
      return beam18_force_test::Values(v);
    };
    const auto x = packet(a[p]), y = packet(bvalues[p]);
    for (unsigned k = 0; k < x.size(); ++k)
      EXPECT_TRUE(fe::shell_startup_detail::SameBits(x[k], y[k]));
    EXPECT_EQ(a[p].stamp.sample_index, bvalues[p].stamp.sample_index);
    EXPECT_TRUE(fe::shell_startup_detail::SameBits(a[p].stamp.time_s, bvalues[p].stamp.time_s));
    for (unsigned q = 0; q < 4; ++q) {
      const auto& first = a[p].point[q]; const auto& second = bvalues[p].point[q];
      EXPECT_EQ(a[p].history.point[q].curve_cursor, bvalues[p].history.point[q].curve_cursor);
      EXPECT_EQ(first.history.curve_cursor, second.history.curve_cursor);
      for (unsigned k = 0; k < 3; ++k)
        EXPECT_TRUE(fe::shell_startup_detail::SameBits(first.history.stress_pa[k], second.history.stress_pa[k]));
      const double fv[]{first.history.plastic_strain, first.plastic_increment, first.yield_stress_pa, first.tangent_factor};
      const double sv[]{second.history.plastic_strain, second.plastic_increment, second.yield_stress_pa, second.tangent_factor};
      for (unsigned k = 0; k < 4; ++k) EXPECT_TRUE(fe::shell_startup_detail::SameBits(fv[k], sv[k]));
    }
    const auto& ad = a[p].diagnostics; const auto& bd = bvalues[p].diagnostics;
    for (unsigned k = 0; k < 2; ++k)
      EXPECT_TRUE(fe::shell_startup_detail::SameBits(ad.internal_work_increment_j[k], bd.internal_work_increment_j[k]));
    EXPECT_TRUE(fe::shell_startup_detail::SameBits(ad.plastic_work_increment_j, bd.plastic_work_increment_j));
  }
}
} // namespace beam18_resident_test
