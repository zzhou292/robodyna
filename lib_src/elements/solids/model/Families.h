// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Model.h"
namespace tl::fea::solids::model_detail {
struct Location { Family family; std::size_t local; };
inline std::size_t Count(ModelInput input) noexcept {
  return input.solid18.size() + input.solid24.size() + input.solid6z.size() +
      input.solid18_law44.size() + input.solid18_law90.size();
}
// Counts only: safe during preflight before any borrowed pointer is read.
inline Location Locate(ModelInput input, std::size_t index) noexcept {
  if (index < input.solid18.size()) return {Family::Solid18, index};
  index -= input.solid18.size();
  if (index < input.solid24.size()) return {Family::Solid24, index};
  index -= input.solid24.size();
  if (index < input.solid6z.size()) return {Family::Solid6z, index};
  index -= input.solid6z.size();
  if (index < input.solid18_law44.size()) return {Family::Solid18Law44, index};
  return {Family::Solid18Law90, index - input.solid18_law44.size()};
}
// All pointer/count pairs and the index extent are preflighted by the caller.
template<class Visitor>
auto Visit(ModelInput input, std::size_t index, Visitor visit) {
  const auto at = Locate(input, index);
  switch (at.family) {
    case Family::Solid18: return visit(input.solid18[at.local], at.local);
    case Family::Solid24: return visit(input.solid24[at.local], at.local);
    case Family::Solid6z: return visit(input.solid6z[at.local], at.local);
    case Family::Solid18Law44: return visit(input.solid18_law44[at.local], at.local);
    case Family::Solid18Law90: return visit(input.solid18_law90[at.local], at.local);
  }
  return visit(input.solid18_law90[at.local], at.local); // Closed internal enum.
}
enum class MaterialLaw { Law36, Law42, Law44, Law90 };
inline MaterialLaw Law(const solid18::Material&) { return MaterialLaw::Law36; }
inline MaterialLaw Law(const solid24::Material&) { return MaterialLaw::Law42; }
inline MaterialLaw Law(const solid18::law44::Material&) { return MaterialLaw::Law44; }
inline MaterialLaw Law(const tl::material::law90::PreparedMaterial&) { return MaterialLaw::Law90; }
inline MaterialLaw LawAt(ModelInput input, std::size_t index) {
  return Visit(input, index, [](const auto& parent, std::size_t) { return Law(parent.material); });
}
} // namespace tl::fea::solids::model_detail
