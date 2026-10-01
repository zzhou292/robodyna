#ifndef ROBODYNA_TESTS_MESH_COMPAT_FIXTURE_H
#define ROBODYNA_TESTS_MESH_COMPAT_FIXTURE_H

#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include <memory>

namespace robodyna::tests::mesh_compat {

inline void Configure(chrono::ChSystemSMC& system) {
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    auto solver = chrono_types::make_shared<chrono::ChSolverMINRES>();
    solver->SetTolerance(1e-12);
    solver->SetMaxIterations(100);
    system.SetSolver(solver);
}

struct SpringMesh {
    std::shared_ptr<chrono::fea::ChMesh> mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    std::shared_ptr<chrono::fea::ChNodeFEAxyz> fixed =
        chrono_types::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(0, 0, 0));
    std::shared_ptr<chrono::fea::ChNodeFEAxyz> moving =
        chrono_types::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(1, 0, 0));
    std::shared_ptr<chrono::fea::ChElementSpring> spring = chrono_types::make_shared<chrono::fea::ChElementSpring>();
    std::shared_ptr<chrono::fea::ChContactSurfaceNodeCloud> contact;
    std::shared_ptr<chrono::fea::ChMeshSurface> surface = chrono_types::make_shared<chrono::fea::ChMeshSurface>();

    explicit SpringMesh(double moving_mass = 3) {
        fixed->SetFixed(true);
        moving->SetMass(moving_mass);
        mesh->AddNode(fixed);
        mesh->AddNode(moving);
        spring->SetNodes(fixed, moving);
        spring->SetSpringCoefficient(12);
        spring->SetDampingCoefficient(0);
        mesh->AddElement(spring);
        mesh->SetAutomaticGravity(false);
        contact = chrono_types::make_shared<chrono::fea::ChContactSurfaceNodeCloud>(
            chrono_types::make_shared<chrono::ChContactMaterialSMC>());
        mesh->AddContactSurface(contact);
        mesh->AddMeshSurface(surface);
    }
};

}  // namespace robodyna::tests::mesh_compat
#endif
