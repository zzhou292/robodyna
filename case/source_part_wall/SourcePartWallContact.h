#pragma once
#include "SourcePartWallSetup.h"
#include "collision/NodalWallContactDevice.h"

namespace crash::cases::source_part_wall {
// Case-level composition of immutable source/wall preparation and the existing
// TL finite-wall contributor. It borrows the sole owner's authenticated views;
// it cannot begin, advance, validate or commit a nodal/material interval.
//
// EvaluateCandidate copies a complete result by value only after every contact
// check and readback succeeds. The case must retain this staged value and may
// publish it ONLY after the same owner and both native histories commit. A later
// rejection/discard cannot overwrite a previously copied accepted result.
class SourcePartWallContact {
  public:
    SourcePartWallContact();
    ~SourcePartWallContact();
    SourcePartWallContact(const SourcePartWallContact&)=delete;
    SourcePartWallContact& operator=(const SourcePartWallContact&)=delete;
    SourcePartWallReport Initialize(const source::SourcePartContactFixture&,const tl::fea::ShellBatchBinding&,
        const tl::fea::NodalStamp&,const double* native_inverse_mass,double measured_initial_kinetic,
        const case_data::CanonicalWall&,const std::string& authenticated_wall_bytes,const SourcePartWallSettings&);
    SourcePartWallReport AssembleAccepted(const tl::fea::NodalAssemblyView&,contact::NodalWallDiagnostics*);
    SourcePartWallReport EvaluateCandidate(const tl::fea::NodalPreparedView&,contact::NodalWallDeviceResults*);
    void DiscardTrial() noexcept;
    bool initialized() const noexcept;
    const SourcePartWallSetup* setup() const noexcept;
    tl::fea::NodalAllocationInfo allocations() const noexcept;
    double stiffness_rate_bound() const noexcept;
    double step_rate_upper() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::cases::source_part_wall
