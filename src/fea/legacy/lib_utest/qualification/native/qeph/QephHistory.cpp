#include "QephHistory.h"

#include <cmath>
#include <cstring>

namespace tl::qualification::qeph {
namespace {
bool SameBits(double a,double b) noexcept {
  static_assert(sizeof(double)==sizeof(std::uint64_t));
  std::uint64_t x,y;
  std::memcpy(&x,&a,sizeof(x)); std::memcpy(&y,&b,sizeof(y));
  return x==y;
}
template<std::size_t N> bool Finite(const std::array<double,N>& values) noexcept {
  for (double x:values) if (!std::isfinite(x)) return false;
  return true;
}
bool Valid(const HistoryValues& values) noexcept {
  return Finite(values.stress)&&Finite(values.material_stress)&&Finite(values.bending_stress)&&
         Finite(values.stabilization)&&Finite(values.strain_curvature)&&Finite(values.internal_work)&&
         std::isfinite(values.thickness)&&values.thickness>0&&
         std::isfinite(values.hourglass_viscous_work)&&values.active==1;
}
}

bool History::matches_reference(const Reference& reference) const noexcept {
  if (!prepared_||!reference.prepared()) return false;
  const auto& actual=reference.data().input;
  const auto& bound=reference_input_;
  for (unsigned i=0;i<4;++i) {
    if (actual.node_ids[i]!=bound.node_ids[i]||
        !SameBits(actual.position[i].x,bound.position[i].x)||
        !SameBits(actual.position[i].y,bound.position[i].y)||
        !SameBits(actual.position[i].z,bound.position[i].z)) return false;
  }
  return SameBits(actual.density,bound.density)&&SameBits(actual.young_modulus,bound.young_modulus)&&
         SameBits(actual.poisson_ratio,bound.poisson_ratio)&&SameBits(actual.thickness,bound.thickness);
}

Status PreparePrescribedHistory(const Reference& reference,const HistoryValues& values,
                                HistoryStamp stamp,History& output) noexcept {
  if (!reference.prepared()) return Status::kInvalidReference;
  if (!Valid(values)||!std::isfinite(stamp.time)||stamp.time<0) return Status::kInvalidInput;
  History candidate;
  candidate.data_=values;
  candidate.stamp_=stamp;
  candidate.reference_input_=reference.data().input;
  candidate.prepared_=true;
  output=candidate;
  return Status::kSuccess;
}

Status InitializeHistory(const Reference& reference,HistoryStamp stamp,History& output) noexcept {
  if (!reference.prepared()) return Status::kInvalidReference;
  HistoryValues initial;
  initial.thickness=reference.data().input.thickness;
  return PreparePrescribedHistory(reference,initial,stamp,output);
}
}  // namespace tl::qualification::qeph
