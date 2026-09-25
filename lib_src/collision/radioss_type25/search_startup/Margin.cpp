// SPDX-License-Identifier: AGPL-3.0-or-later
// Original I25BUC_VOX1 and margin_reduction operation/row order.
#include "Values.h"
namespace tlfea::contact::radioss_type25::search_startup::detail {
namespace {
Report Small(const Input& in, const Vector* x, double multiplier, double gap,
    Limits limits, double& mean) noexcept {
  const double saved = mean;
  std::size_t selected = 0;
  double largest = -1;
  for (std::size_t i = 0; i < in.main_count; ++i) {
    const auto b = Bounds(x, in.topology.mains[i]);
    const double volume = (b.maximum.x-b.minimum.x)*(b.maximum.y-b.minimum.y)*
        (b.maximum.z-b.minimum.z);
    if (!std::isfinite(volume)) return {Status::NonfiniteResult, i};
    if (volume > largest) { largest = volume; selected = i; }
  }
  const auto basis = Bounds(x, in.topology.mains[selected]);
  auto bounds = [&](double padding) noexcept {
    return Box{{basis.minimum.x-padding,basis.minimum.y-padding,basis.minimum.z-padding},
        {basis.maximum.x+padding,basis.maximum.y+padding,basis.maximum.z+padding}};
  };
  // Selected INTTH0 and absent gap-load: native MAX(gap+0,0).
  const double padding = std::max(gap + 0., 0.);
  const auto initial = bounds(padding);
  if (!Finite(initial)) return {Status::NonfiniteResult};
  std::size_t minimum_count = 0;
  for (std::size_t i = 0; i < in.secondary_count; ++i) {
    if (in.secondary[i].stiffness == 0.) continue;
    if (Inside(x[in.secondary[i].node], initial)) ++minimum_count;
  }
  const auto threshold = std::max(std::size_t{1000}+minimum_count, in.secondary_count/100);
  std::size_t count = in.secondary_count, iterations = 0;
  while (count > threshold) {
    if (++iterations > limits.max_margin_iterations) return {Status::ResourceLimit};
    // Keep the source association: basis +/- multiplier*DD0 +/- gap padding.
    const double extra = multiplier*mean;
    Box current{{basis.minimum.x-extra-padding,basis.minimum.y-extra-padding,basis.minimum.z-extra-padding},
        {basis.maximum.x+extra+padding,basis.maximum.y+extra+padding,basis.maximum.z+extra+padding}};
    if (!Finite(current)) return {Status::NonfiniteResult};
    count = 0;
    // This second count intentionally includes zero-stiffness secondaries.
    for (std::size_t i = 0; i < in.secondary_count; ++i)
      if (Inside(x[in.secondary[i].node], current)) ++count;
    if (count <= threshold) break;
    const double next = mean * .75;
    if (!std::isfinite(next)) return {Status::NonfiniteResult};
    const bool changed = next != mean;
    mean = next;
    // Native unsuffixed REAL literal is converted to the MYREAL8 operand.
    if (mean < saved*static_cast<double>(.001f)) break;
    if (!changed) return {Status::NoProgress};
  }
  return {Status::Ok};
}
}
Report Margin(const Input& in, const Vector* x, double multiplier, double gap,
    Limits limits, double& mean, double& margin) noexcept {
  double total = 0;
  for (std::size_t i = 0; i < in.main_count; ++i) {
    const auto& m = in.topology.mains[i];
    const double d1 = Distance(x[m.nodes[0]],x[m.nodes[1]]);
    const double d2 = Distance(x[m.nodes[0]],x[m.nodes[3]]);
    const double d3 = Distance(x[m.nodes[2]],x[m.nodes[1]]);
    const double d4 = Distance(x[m.nodes[3]],x[m.nodes[2]]);
    total = total + (d1+d2+d3+d4);
    if (!std::isfinite(total)) return {Status::NonfiniteResult, i};
  }
  mean = total/static_cast<double>(in.main_count)/4.;
  if (in.main_count <= 3) {
    const auto report = Small(in,x,multiplier,gap,limits,mean);
    if (report.status != Status::Ok) return report;
  }
  margin = multiplier*mean;
  if (!std::isfinite(margin)) return {Status::NonfiniteResult};
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
