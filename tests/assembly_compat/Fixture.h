// The same legacy API fixture is used before and after the implementation rename.
#pragma once

#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChNodeFEAxyz.h"

#include <memory>

namespace robodyna::assembly_compat {
struct Fixture {
    chrono::ChSystemSMC system;
    std::shared_ptr<chrono::ChAssembly> assembly = std::make_shared<chrono::ChAssembly>();
    std::shared_ptr<chrono::ChBody> body = std::make_shared<chrono::ChBody>();
    std::shared_ptr<chrono::ChBody> fixed = std::make_shared<chrono::ChBody>();
    std::shared_ptr<chrono::ChShaft> shaft = std::make_shared<chrono::ChShaft>();
    std::shared_ptr<chrono::fea::ChMesh> mesh = std::make_shared<chrono::fea::ChMesh>();
    std::shared_ptr<chrono::fea::ChNodeFEAxyz> fixed_node =
        std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(0, 0, 0));
    std::shared_ptr<chrono::fea::ChNodeFEAxyz> moving_node =
        std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(1, 0, 0));
    std::shared_ptr<chrono::fea::ChLinkNodeFrame> link = std::make_shared<chrono::fea::ChLinkNodeFrame>();

    Fixture() {
        system.SetNumThreads(1, 1, 1);
        system.SetGravitationalAcceleration({0, 0, 0});
        system.Add(assembly);  // Legacy Add methods require a containing system.
        body->SetMass(2);
        body->SetSleepingAllowed(false);
        body->SetPos({1, 0, 0});
        fixed->SetFixed(true);
        shaft->SetInertia(3);
        fixed_node->SetFixed(true);
        moving_node->SetMass(1);
        mesh->AddNode(fixed_node);
        mesh->AddNode(moving_node);
        mesh->SetAutomaticGravity(false);
        link->Initialize(moving_node, body);
        // Generic Add must preserve admission into each distinct collection.
        assembly->Add(body);
        assembly->Add(fixed);
        assembly->Add(shaft);
        assembly->Add(mesh);
        assembly->Add(link);
    }
};
}  // namespace robodyna::assembly_compat
