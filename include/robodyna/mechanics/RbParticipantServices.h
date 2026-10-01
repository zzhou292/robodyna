#ifndef ROBODYNA_MECHANICS_RBPARTICIPANTSERVICES_H
#define ROBODYNA_MECHANICS_RBPARTICIPANTSERVICES_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"

namespace robodyna::mechanics {

/// Borrowed access to the existing owner's participant environment.
/// This interface owns no state, clock, participant collection or solver.
/// The owner must outlive every attached participant, as with GetSystem().
class ChApi RbParticipantServices {
  public:
    virtual const chrono::ChVector3d& GetParticipantGravity() const = 0;
    virtual int GetParticipantAssemblyThreads() const = 0;
    virtual void InvalidateParticipantInitializationAndUpdate() = 0;
    virtual void InvalidateParticipantUpdate() = 0;

  protected:
    // A service reference is borrowed; deletion through this interface is forbidden.
    ~RbParticipantServices() = default;
};

}  // namespace robodyna::mechanics

#endif
