// SPDX-License-Identifier: AGPL-3.0-or-later
// Source phase order: CINMAS/C3INMAS -> SPMD_MSIN -> ASSTIFI;
// I25STI3 secondary gap mask and I25INI_GAP_N main nodal gap distribution.
#include "../../RadiossType25ShellSource.h"
#include "Layout.h"
#include "Values.h"
#include "../search/Ranges.h"
#include <cstring>
#include <new>
namespace tlfea::contact::radioss_type25::source_shells {
Report Preflight(const Input& input, Limits limits, Forecast& output) noexcept {
  detail::Layout layout;
  const auto report=detail::Prepare(input,limits,layout);
  if (report.status==Status::Ok) output=layout.forecast;
  return report;
}

Report Build(const Input& input, Limits limits, void* scratch, std::size_t bytes, Output output) noexcept {
  detail::Layout layout;
  const auto admitted=detail::Prepare(input,limits,layout);
  if (admitted.status!=Status::Ok) return admitted;
  if (!detail::SeparateStorage(input,layout,scratch,bytes,output)) return {Status::InvalidInput};
  using tl::util::ArenaPointer;
  auto* nodes=::new (static_cast<void*>(ArenaPointer<NodeFields>(scratch,layout.nodes))) NodeFields[input.node_count]{};
  auto* selected=::new (static_cast<void*>(ArenaPointer<unsigned char>(scratch,layout.selected_shells))) unsigned char[input.shell_count]{};
  auto* primaries=ArenaPointer<double>(scratch,layout.primaries);
  auto* secondary=ArenaPointer<SecondaryFields>(scratch,layout.secondary);
  if (input.primary_count) ::new (static_cast<void*>(primaries)) double[input.primary_count]{};
  if (input.secondary_count) ::new (static_cast<void*>(secondary)) SecondaryFields[input.secondary_count]{};
  namespace c=coefficient_detail;
  for (std::size_t i=0;i<input.shell_count;++i) {
    const auto& shell=input.shells[i];
    // These are physical structural thicknesses, not contact gap overrides.
    const double contribution=shell.young*shell.structural_thickness;
    if (!c::Finite(contribution)) { Report bad{Status::NonfiniteResult};bad.physical_shell=i;return bad; }
    const double half_gap=detail::HalfGap(shell,input.profile.input_thickness_mode);
    for (unsigned slot=0;slot<detail::Slots(shell.layout);++slot) {
      const auto ordinal=shell.nodes[slot];auto& node=nodes[ordinal];
      const double sum=node.young_thickness_sum+contribution;
      if (!c::Finite(sum)) { Report bad{Status::NonfiniteResult};bad.physical_shell=i;bad.node=ordinal;return bad; }
      node.young_thickness_sum=sum;
      ++node.shell_incidence_count; // Bounded by admitted shell_count < INT_MAX.
      node.unscaled_half_gap=c::Max(node.unscaled_half_gap,half_gap);
    }
  }
  for (std::size_t i=0;i<input.primary_count;++i) {
    const auto shell_ordinal=input.primary_shells[i];
    if (selected[shell_ordinal]) { Report bad{Status::InvalidInput};bad.primary=i;return bad; }
    selected[shell_ordinal]=1;
    const auto& shell=input.shells[shell_ordinal];
    for (unsigned slot=0;slot<detail::Slots(shell.layout);++slot) nodes[shell.nodes[slot]].on_main_surface=true;
    NativeShellMainCoefficientInput main;
    main.face=MainFaceKind::OrdinaryExterior;main.layout=shell.layout;
    main.property_type=input.profile.property_type;
    main.input_thickness_mode=input.profile.input_thickness_mode;
    main.scale=input.profile.stiffness_scale;main.young=shell.young;
    main.element_thickness=shell.element_thickness;main.property_thickness=shell.property_thickness;
    NativeScalarCoefficient result;
    if (EvaluateNativeShellMainCoefficient(main,&result)!=CoefficientStatus::Ok) {
      Report bad{Status::NonfiniteResult};bad.primary=i;return bad;
    }
    primaries[i]=result.value;
  }
  for (std::size_t i=0;i<input.node_count;++i) {
    auto& node=nodes[i];
    NativeAccumulatedNodalCoefficients accumulated;
    accumulated.young_thickness_sum=node.young_thickness_sum;
    accumulated.shell_incidence_count=node.shell_incidence_count;
    NativeNodalCoefficientResult result;
    if (FinalizeNativeNodalCoefficient(accumulated,&result)!=CoefficientStatus::Ok) {
      Report bad{Status::NonfiniteResult};bad.node=i;return bad;
    }
    node.stiffness=result.stiffness;
    if (node.on_main_surface) {
      const double scaled=input.profile.gap_scale*node.unscaled_half_gap;
      if (!c::Finite(scaled)) { Report bad{Status::NonfiniteResult};bad.node=i;return bad; }
      node.main_gap=c::Min(scaled,input.profile.maximum_main_gap);
    }
  }
  for (std::size_t i=0;i<input.secondary_count;++i) {
    const auto& source=input.secondary[i];const auto& node=nodes[source.node];
    NativeSecondaryCoefficientInput coefficient;
    coefficient.existing=source.existing_coefficient;coefficient.global_stiffness=node.stiffness;
    coefficient.scale=input.profile.stiffness_scale;
    NativeScalarCoefficient result;
    if (EvaluateNativeSecondaryCoefficient(coefficient,&result)!=CoefficientStatus::Ok) {
      Report bad{Status::NonfiniteResult};bad.secondary=i;return bad;
    }
    secondary[i].stiffness=result.value;
    // Native ILEV1 first masks nodal shell thickness to selected shell mains,
    // then applies GAPSCALE and the distinct secondary cap.
    const double half_gap=node.on_main_surface?node.unscaled_half_gap:0.;
    const double scaled=input.profile.gap_scale*half_gap;
    if (!c::Finite(scaled)) { Report bad{Status::NonfiniteResult};bad.secondary=i;return bad; }
    secondary[i].gap=c::Min(scaled,input.profile.maximum_secondary_gap);
  }
  std::memcpy(output.nodes,nodes,layout.nodes.bytes);
  if (input.primary_count) std::memcpy(output.primary_stiffness,primaries,layout.primaries.bytes);
  if (input.secondary_count) std::memcpy(output.secondary,secondary,layout.secondary.bytes);
  return {Status::Ok};
}

Report MainGaps(const NodeFields* fields, std::size_t count, const std::uint32_t (&nodes)[4],
    MainGapFields* output) noexcept {
  if (!count || count>Limits{}.nodes || !search::detail::Span(fields,count) ||
      !search::detail::Span(output,std::size_t{1})) return {Status::InvalidInput};
  MainGapFields result;
  for (unsigned slot=0;slot<4;++slot) {
    if (nodes[slot]>=count || !fields[nodes[slot]].on_main_surface ||
        !coefficient_detail::Nonnegative(fields[nodes[slot]].main_gap)) return {Status::InvalidInput};
    result.corner[slot]=fields[nodes[slot]].main_gap;
    result.maximum=coefficient_detail::Max(result.maximum,result.corner[slot]);
  }
  *output=result;return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::source_shells
