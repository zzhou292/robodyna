// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
//     - collisions and contacts
//     - using Irrlicht to display objects.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"
#ifdef ROBODYNA_CAPTURE_DEMO
    #include "examples/support/CaptureSession.h"
    #include "examples/mbd/Telemetry.h"
#endif

#ifdef CHRONO_COLLISION
    #include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#if defined(CHRONO_VSG) || defined(ROBODYNA_CAPTURE_DEMO)
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;

void AddFallingItems(ChSystemNSC& sys) {
    // Shared contact materials for falling objects
    auto sph_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    sph_mat->SetFriction(0.2f);
    auto box_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto cyl_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create falling rigid bodies (spheres and boxes etc.)
    for (int bi = 0; bi < 29; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(1.1,     // radius size
                                                                      1000,    // density
                                                                      sph_mat  // contact material
        );
        sphereBody->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        sphereBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sys.Add(sphereBody);

        auto boxBody = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5,  // x,y,z size
                                                                100,            // density
                                                                box_mat         // contact material
        );
        boxBody->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        boxBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_bluewhite.png"));
        sys.Add(boxBody);

        auto cylBody = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y,  //
                                                                     0.75, 0.5,  // radius, height
                                                                     100,        // density
                                                                     cyl_mat     // contact material
        );
        cylBody->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        cylBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"));
        sys.Add(cylBody);
    }
}

std::shared_ptr<ChBody> AddContainer(ChSystemNSC& sys) {
    // Contact and visualization materials for container
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    ground_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    // Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, ground_mat);
    floorBody->SetPos(ChVector3d(0, -5, 0));
    floorBody->SetFixed(true);
    floorBody->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, ground_mat);
    wallBody1->SetPos(ChVector3d(-10, 0, 0));
    wallBody1->SetFixed(true);
    wallBody1->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, ground_mat);
    wallBody2->SetPos(ChVector3d(10, 0, 0));
    wallBody2->SetFixed(true);
    wallBody2->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, ground_mat);
    wallBody3->SetPos(ChVector3d(0, 0, -10));
    wallBody3->SetFixed(true);
    wallBody3->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, ground_mat);
    wallBody4->SetPos(ChVector3d(0, 0, 10));
    wallBody4->SetFixed(true);
    wallBody4->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody4);

    // Add the rotating mixer
    auto mixer_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mixer_mat->SetFriction(0.4f);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                                 4000,      // density
                                                                 mixer_mat  // contact material
    );
    rotatingBody->SetPos(ChVector3d(0, -1.6, 0));
    rotatingBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(rotatingBody);

    // .. a motor between mixer and truss

    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)));
    auto mfun = chrono_types::make_shared<ChFunctionConst>(CH_PI / 4.0);  // speed 45 deg/s
    my_motor->SetSpeedFunction(mfun);
    sys.AddLink(my_motor);

    return rotatingBody;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nRobodyna demo; inherited Chrono version: " << CHRONO_VERSION << std::endl;

#ifdef ROBODYNA_CAPTURE_DEMO
    try {
    const auto capture_options = robodyna::examples::ReadCaptureOptions(
        argc, argv, "src/compatibility/chrono/src/demos/mbs/demo_MBS_collisionNSC.cpp", 0.003);
    robodyna::examples::CaptureSession capture(capture_options);
    constexpr std::uint32_t capture_seed = 20260930;
    ChRandom::SetSeed(capture_seed);
#endif

    // Create the physical system
    ChSystemNSC sys;
    sys.SetGravityY();
    sys.SetCollisionSystemType(collision_type);

    // Settings specific to Chrono multicore collision system
    if (collision_type == ChCollisionSystem::Type::MULTICORE) {
#ifdef CHRONO_COLLISION
        auto collsys = std::static_pointer_cast<ChCollisionSystemMulticore>(sys.GetCollisionSystem());
        // Change the default number of broadphase bins
        collsys->SetBroadphaseGridResolution(ChVector3i(10, 10, 2));
        // Change default narrowphase algorithm
        ////collsys->SetNarrowphaseAlgorithm(ChNarrowphase::Algorithm::MPR);
        collsys->SetEnvelope(0.005);
        // Enable active bounding box
        collsys->EnableActiveBoundingBox(ChVector3d(-10, -10, -20), ChVector3d(+10, +10, +10));
        // Set number of threads used by the collision detection system
        collsys->SetNumThreads(4);
#endif
    }

    // Add fixed and moving bodies
    auto mixer = AddContainer(sys);
    AddFallingItems(sys);

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#if !defined(CHRONO_VSG) && !defined(ROBODYNA_CAPTURE_DEMO)
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
#ifdef ROBODYNA_CAPTURE_DEMO
    if (!capture_options.headless) {
#endif
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Robodyna | Rigid-body collisions");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 14, -20));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#if defined(CHRONO_VSG) || defined(ROBODYNA_CAPTURE_DEMO)
#ifdef ROBODYNA_CAPTURE_DEMO
            auto vis_vsg = chrono_types::make_shared<robodyna::examples::CaptureVisual>();
#else
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
#endif
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("Robodyna | Rigid-body collisions");
            vis_vsg->AddCamera(ChVector3d(0, 18, -20));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyTexture(SkyMode::BOX);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
#ifdef ROBODYNA_CAPTURE_DEMO
            capture.ConfigureVisual(*vis_vsg);
            vis_vsg->Initialize();
#else
            vis_vsg->Initialize();
#endif

            vis = vis_vsg;
#endif
            break;
        }
    }
#ifdef ROBODYNA_CAPTURE_DEMO
    }
#endif

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(50);

    ////sys.SetSleepingAllowed(true);

    // Simulation loop
    double step_size = 0.003;
#ifdef ROBODYNA_CAPTURE_DEMO
    robodyna::examples::mbd::CollisionTelemetry telemetry(sys, capture_seed);
    telemetry.Observe(sys);
    capture.State(0, sys, capture_options.headless ? nullptr : vis.get());
    for (std::uint64_t step = 1; step <= capture_options.steps; ++step) {
        robodyna::examples::Require(sys.DoStepDynamics(step_size) != 0, "Original NSC dynamics step failed");
        telemetry.Observe(sys);
        capture.State(step, sys, capture_options.headless ? nullptr : vis.get());
    }
    capture.Finish(sys, telemetry.Summary());
#else
    ChRealtimeStepTimer rt;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);

        ////std::cout << std::endl;
        ////auto frc = mixer->GetAppliedForce();
        ////auto trq = mixer->GetAppliedTorque();
        ////std::cout << sys.GetChTime() << "  force: " << frc << "  torque: " << trq << std::endl;
        ////auto c_frc = mixer->GetContactForce();
        ////auto c_trq = mixer->GetContactTorque();
        ////std::cout << sys.GetChTime() << "  ct force: " << c_frc << "  ct torque: " << c_trq << std::endl;

        rt.Spin(step_size);
    }
#endif

    return 0;
#ifdef ROBODYNA_CAPTURE_DEMO
    } catch (const std::exception& error) {
        std::cerr << "robodyna original collisionNSC: " << error.what() << '\n';
        return 1;
    }
#endif
}
