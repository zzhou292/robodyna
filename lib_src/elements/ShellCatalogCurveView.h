// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPlasticityBinding.h"

namespace tl::fea::shell_batch_plasticity_detail {
// Internal immutable pool view. The complete catalog remains the sole curve
// offset authority; consumers copy its native values and rebase table pointers.
struct CatalogCurveView {
  const double* x=nullptr;
  const double* y=nullptr;
  std::size_t count=0;
  explicit CatalogCurveView(const ShellBatchPlasticityBinding& catalog) noexcept
      :x(catalog.data_.curve_x.data()),y(catalog.data_.curve_y.data()),
       count(catalog.data_.point_count) {}
  bool Offset(const sections::PointParameters& parameters,std::size_t& offset) const noexcept {
    if(parameters.hardening==material::ShellPlasticityHardeningKind::LinearLaw44) {
      if(parameters.curve.count||parameters.curve.plastic_strain||parameters.curve.yield_stress_pa) return false;
      offset=NoShellBindingNode;
      return true;
    }
    // Parameters() only returns views into this catalog. Search addresses
    // before subtraction, so even malformed views cannot create undefined C++.
    for(std::size_t i=0;i<count;++i) {
      if(parameters.curve.plastic_strain!=x+i) continue;
      if(parameters.curve.yield_stress_pa!=y+i||parameters.curve.count<2||
          parameters.curve.count>count-i) return false;
      offset=i;
      return true;
    }
    return false;
  }
};
} // namespace tl::fea::shell_batch_plasticity_detail
