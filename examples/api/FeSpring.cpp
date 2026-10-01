// Minimal headless FE node/spring example using only Robodyna public includes.
#include "robodyna/core/RbTypes.h"
#include "robodyna/core/RbVector3.h"
#include "robodyna/fea/RbElementSpring.h"
#include "robodyna/fea/RbMesh.h"
#include "robodyna/fea/RbNodeFEAxyz.h"
#include "robodyna/numerics/RbSolver.h"
#include "robodyna/numerics/RbTimestepper.h"
#include "robodyna/simulation/RbSystemSMC.h"
#include "SpringCheck.h"

int main() {
    namespace rd = robodyna;
    rd::simulation::RbSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(rd::numerics::RbTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.SetSolverType(rd::numerics::RbSolver::Type::MINRES);

    auto mesh = rd::core::make_shared<rd::fea::RbMesh>();
    auto anchor = rd::core::make_shared<rd::fea::RbNodeFEAxyz>(rd::core::RbVector3d(0, 0, 0));
    auto node = rd::core::make_shared<rd::fea::RbNodeFEAxyz>(rd::core::RbVector3d(1, 0, 0));
    anchor->SetFixed(true);
    node->SetMass(3);
    auto spring = rd::core::make_shared<rd::fea::RbElementSpring>();
    spring->SetNodes(anchor, node);
    spring->SetSpringCoefficient(12);
    spring->SetDampingCoefficient(0);
    mesh->AddNode(anchor);
    mesh->AddNode(node);
    mesh->AddElement(spring);
    mesh->SetAutomaticGravity(false);
    system.AddMesh(mesh);
    // Keep the reference geometry at length 1; displace the current state only.
    node->SetPos({1.1, 0, 0});

    for (unsigned step = 0; step < 1000; ++step) {
        if (!system.DoStepDynamics(1e-4, false))
            return 1;
    }
    return rd::examples::api::ReportSpring("finite_element_spring", system.GetChTime(),
                                          node->GetPos().x(), node->GetPosDt().x());
}
