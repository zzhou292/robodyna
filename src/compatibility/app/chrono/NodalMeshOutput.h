#pragma once

#include "AcceptedSurfaceMesh.h"
#include "lib_src/solvers/FENodalState.h"
#include "NodalCaptureLimits.h"
#include <array>
#include <vector>

namespace crash::visual {
enum class NodalOutputTiming { CollocatedOnly, StaggeredHalfKick };

// Accepted-only application boundary for the TL physical-node owner. The caller
// chooses output cadence; Publish never advances time or performs a trial. It
// receives a live owner each time and checks the identity captured at setup, so
// no owner pointer or borrowed device memory is retained between calls.
// Setup/publication/rendering are externally serialized. Capture storage follows
// an explicit startup node/host-byte limit (legacy default 128, opt-in 2048). No shell-qualification library is linked.
// The two-argument Initialize preserves the collocated-only protocol. Staggered
// output requires explicit opt-in and binds the owner's scheme, fixed dt, node
// count and rotation availability. Positions/quaternions are accepted endpoint
// fields; raw velocity/angular velocity retain the actual stamp's velocity_time
// and phase (initially collocated, then the previous midpoint). No synchronous
// velocity is reconstructed. Reaction metadata is retained; reaction arrays are
// not captured. This is an in-memory output boundary, not an archive schema.
//
// stamp()/fields() become available only after successful mesh publication.
// Translation-only owners expose null orientation/angular-velocity pointers.
// Capture storage is preallocated; failed publication preserves both the visible
// mesh and all exposed fields. Borrowed host views remain valid until the next
// successful Publish or destruction; callers must not retain them across either.
class NodalMeshOutput {
 public:
  Report Initialize(const tl::fea::FENodalState&, const Binding&);
  Report Initialize(const tl::fea::FENodalState&, const Binding&, NodalOutputTiming);
  Report Initialize(const tl::fea::FENodalState&, const Binding&, NodalOutputTiming, NodalCaptureLimits);
  Report Publish(tl::fea::FENodalState&);
  std::size_t capture_bytes() const noexcept { return capture_bytes_; }
  const AcceptedSurfaceMesh& surface() const noexcept { return surface_; }
  const tl::fea::NodalStamp* stamp() const noexcept { return available_ ? &stamp_ : nullptr; }
  tl::fea::HostNodalKinematicsView fields() const noexcept;
 private:
  struct Capture {
    std::vector<double> values;
    double* position() noexcept { return values.data(); }
    double* velocity(std::size_t n) noexcept { return values.data() + 3*n; }
    double* omega(std::size_t n) noexcept { return values.data() + 6*n; }
    double* orientation(std::size_t n) noexcept { return values.data() + 9*n; }
  };
  AcceptedSurfaceMesh surface_;
  Identity identity_{};
  tl::fea::NodalStamp bound_{}, stamp_{};
  NodalOutputTiming timing_ = NodalOutputTiming::CollocatedOnly;
  std::array<Capture, 2> captures_{};
  std::size_t capture_bytes_ = 0;
  unsigned published_ = 0;
  bool available_ = false;
};
}  // namespace crash::visual
