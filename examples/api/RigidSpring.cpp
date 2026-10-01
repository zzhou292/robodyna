// Minimal headless body/spring example using only Robodyna public includes.
#include "robodyna/core/RbTypes.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/mbd/RbBodyEasy.h"
#include "robodyna/mbd/RbLinkTSDA.h"
#include "robodyna/numerics/RbTimestepper.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include "SpringCheck.h"

int main() {
    namespace rd = robodyna;
    rd::simulation::RbSystemNSC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(rd::numerics::RbTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto anchor = rd::core::make_shared<rd::mbd::RbBody>();
    anchor->SetFixed(true);
    system.AddBody(anchor);
    // A unit cube of density 3 has mass 3. No renderer or collision is needed.
    auto body = rd::core::make_shared<rd::mbd::RbBodyEasyBox>(1, 1, 1, 3, false, false);
    body->SetPos({1.1, 0, 0});
    body->SetSleepingAllowed(false);
    system.AddBody(body);
    auto spring = rd::core::make_shared<rd::mbd::RbLinkTSDA>();
    spring->Initialize(anchor, body, true, {0, 0, 0}, {0, 0, 0});
    spring->SetRestLength(1);
    spring->SetSpringCoefficient(12);
    spring->SetDampingCoefficient(0);
    system.AddLink(spring);

    for (unsigned step = 0; step < 1000; ++step) {
        if (!system.DoStepDynamics(1e-4, false))
            return 1;
    }
    return rd::examples::api::ReportSpring("rigid_tsda", system.GetChTime(),
                                          body->GetPos().x(), body->GetPosDt().x());
}
