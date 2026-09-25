// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <vector>
#include <utility>
namespace type25_geometry_test {
inline n::NativeRawGeometryResult Raw(double penetration, double stiffness = 400) {
  n::NativeRawGeometryResult r;
  r.key=Quad().key;r.selection_code=1;r.normal={0,0,1};
  for(auto& w:r.weights)w=.25;
  r.geometric_penetration=penetration;r.gap=.6;r.distance=.2;r.incoming_stiffness=stiffness;
  return r;
}
struct HistoryFixture {
  std::vector<n::NativeRawGeometryResult> geometry;
  std::vector<n::NativeGeometryHistory> selected, staged, scratch_rows;
  std::vector<n::NativeGeometryFinalResult> results, scratch_results;
  HistoryFixture(std::vector<n::NativeRawGeometryResult> g,
      std::vector<n::NativeGeometryHistory> h)
      :geometry(std::move(g)),selected(std::move(h)),staged(selected),scratch_rows(selected.size()),
       results(geometry.size()),scratch_results(geometry.size()) {
    for(auto& r:staged)r.row.penetration_auxiliary=913;
    for(auto& r:results){r.geometry=Raw(.75,987);r.penetration=912;}
  }
  n::GeometryHistoryBatch View() {
    return {geometry.data(),geometry.size(),selected.data(),selected.size(),
      staged.data(),results.data(),scratch_rows.data(),scratch_rows.size(),
      scratch_results.data(),scratch_results.size()};
  }
};
} // namespace type25_geometry_test
