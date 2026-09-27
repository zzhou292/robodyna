// Generated from complete pinned pre-change loops; no predicate substitution.
#pragma once
#include "Fixture.h"
namespace t3_compact_test::serial {
using namespace tl::fea;using namespace tl::fea::t3;
using namespace tl::fea::shell_batch_plasticity_detail;
struct FailureSource { ShellFailurePolicy policy; };
inline bool ReadLaw(const Fixture& f,std::size_t p,ShellSectionLaw* value) {
  if(p==f.missing_law)return false;*value=f.laws[p];return true;
}
inline bool ReadParameters(const Fixture& f,std::size_t p,sections::PointParameters* value) {
  if(p==f.missing_parameter)return false;*value=f.parameters[p];return true;
}
inline SetupReport Points(Fixture& f) {  for (std::size_t e = 0; e < f.size(); ++e) {
    ShellSectionLaw law = ShellSectionLaw::Unspecified;
    if (!ReadLaw(f,e,&law)) {
      return {SetupStatus::InvalidInput, "One-point readback source identity is incomplete"};
    }
    if (law != ShellSectionLaw::Law44Nip1) continue;
    sections::PointParameters parameters;
    if (!ReadParameters(f,e,&parameters) ||
        !ValidOnePointState(f.points[e], parameters, f.time)) {
      return {SetupStatus::NonfiniteResult, "One-point saved/current/failure state is invalid"};
    }
  }
  return {SetupStatus::Success, "OK"};
}
inline SetupReport Mixed(Fixture& f) {  const auto* one_point=f.has_point?f.points.data():nullptr;
  // Complete finite/availability preflight precedes even private output staging.
  for(std::size_t e=0;e<f.size();++e) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if(!ReadLaw(f,e,&law))return {SetupStatus::InvalidInput,"Mixed section source identity is unavailable"};
    if(law==ShellSectionLaw::LayeredLaw1Nip3) {
      if(!FiniteSection(f.elastic[e]))return {SetupStatus::NonfiniteResult,"Nonfinite elastic section history"};
    } else if(law==ShellSectionLaw::LayeredLaw44Nip3) {
      if(!FiniteSection(f.plastic[e]))return {SetupStatus::NonfiniteResult,"Nonfinite plastic section history"};
    } else if(law==ShellSectionLaw::Law44Nip1) {
      // The optional owner stages and validates its complete payload first.
      if(!one_point||ShellBindingFamily::T3!=ShellBindingFamily::T3)
        return {SetupStatus::InvalidInput,"One-point history is unavailable"};
    } else if((law!=ShellSectionLaw::RigidSkin&&law!=ShellSectionLaw::GlobalLaw1Npt0)||!f.execution) {
      return {SetupStatus::InvalidInput,"Unsupported section readback law"};
    }
  }
  for(std::size_t e=0;e<f.size();++e) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;ReadLaw(f,e,&law);
    if(law==ShellSectionLaw::LayeredLaw1Nip3)
      f.sections[e]=ShellBatchLayeredSection::Elastic(f.elastic[e]);
    else if(law==ShellSectionLaw::LayeredLaw44Nip3)
      f.sections[e]=ShellBatchLayeredSection::Plastic(f.plastic[e]);
    else if(law==ShellSectionLaw::Law44Nip1)
      f.sections[e]=ShellBatchLayeredSection::OnePoint(one_point[e]);
    else if(law==ShellSectionLaw::GlobalLaw1Npt0)f.sections[e]=ShellBatchLayeredSection::GlobalLaw1();
    else f.sections[e]=ShellBatchLayeredSection::RigidSkin();
  }
  return {SetupStatus::Success,"OK"};
}
inline SetupReport Failure(Fixture& f) {  for (std::size_t e = 0; e < f.size(); ++e) {
    const FailureSource declared{f.policies[e]}; const auto* source=e==f.missing_failure?nullptr:&declared;
    if (f.sections[e].law() == ShellSectionLaw::Law44Nip1) {
      if (!source || source->policy != ShellFailurePolicy::ConstantAllPoints ||
          !f.sections[e].one_point() || !ValidFailureEncoding(f.failure[e]) ||
          !ValidFailureState(f.failure[e], ShellFailurePolicy::None, nullptr, f.time)) {
        return {SetupStatus::NonfiniteResult, "One-point row acquired a three-point failure state"};
      }
      continue; // The genuine point/failure packet was fully validated first.
    }
    if (!source || f.failure[e].policy() != source->policy || !ValidFailureEncoding(f.failure[e]) ||
        !ValidFailureState(f.failure[e], source->policy, f.sections[e].plastic(), f.time)) {
      return {SetupStatus::NonfiniteResult,
              "Failure sidecar state disagrees with its declared policy/saved section"};
    }
  }
  return {SetupStatus::Success, "OK"};
}
inline BatchReport Force(Fixture& f) {  for (std::size_t parent=0;parent<f.size();++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!ReadLaw(f,parent,&law) ||
        !mapped::ValidResult(f.elements[parent].reference,f.forces[parent],f.force_time,f.force_epoch,
            law==ShellSectionLaw::RigidSkin)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force cache differs from its source/endpoint role",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
inline BatchReport Roles(Fixture& f) {  for (std::size_t parent=0;parent<f.size();++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!ReadLaw(f,parent,&law) || f.sections[parent].law()!=law) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 typed section role differs",static_cast<std::uint32_t>(parent)};
    }
    // True NIP1 failure belongs to the point payload, not the reserved NIP3 slot.
    const bool active=f.sections[parent].one_point()?f.sections[parent].one_point()->point.failure.history.point_active:
        f.failure[parent].active;
    if (f.forces[parent].proposed_history.data().active!=(active?1:0)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force and failure activity differ",static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
inline BatchReport PointIdentity(Fixture& f) {  for (std::size_t e = 0; e < f.size(); ++e) {
    const auto* point = f.sections[e].one_point();
    if (!point) continue;
    const auto& history = f.forces[e].proposed_history;
    const auto& reference = f.elements[e].reference;
    const auto& saved = point->point.saved;
    sections::PointParameters parameters;
    if (!ReadParameters(f,e,&parameters) ||
        !history.matches_reference(reference) || history.stamp().time != f.time ||
        history.stamp().sample_index != f.epoch ||
        !detail::SameHistoryBits(history.data().thickness, point->point.reported_thickness_m) ||
        !one_point_detail::ValidValues({history.data(), saved, point->point.failure.history,
            point->cumulative_plastic_work_J}, parameters, f.time)) {
      return {BatchStatus::NonfiniteResult, "One-point state differs from its actual shell slab/history identity",
              static_cast<std::uint32_t>(e)};
    }
  }
  return {BatchStatus::Success, "OK"};
}
} // namespace t3_compact_test::serial
