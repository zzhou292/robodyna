#pragma once

#include "AcceptedSurfaceMesh.h"
#include <array>

namespace tl::fea { class FENodalState; }

namespace crash::visual {
// Accepted-only application boundary for the TL physical-node owner. The caller
// chooses output cadence; Publish never advances time or performs a trial. It
// receives a live owner each time and checks the identity captured at setup, so
// no owner pointer or borrowed device memory is retained between calls.
// Setup/publication/rendering are externally serialized. The bounded TL owner
// currently admits at most 64 nodes. No shell-qualification library is linked.
class NodalMeshOutput {
 public:
  Report Initialize(const tl::fea::FENodalState&, const Binding&);
  Report Publish(tl::fea::FENodalState&);
  const AcceptedSurfaceMesh& surface() const noexcept { return surface_; }
 private:
  AcceptedSurfaceMesh surface_;
  Identity identity_{};
  std::size_t node_count_ = 0;
  std::array<double, 64 * 3> position_{}, velocity_{};
};
}  // namespace crash::visual
