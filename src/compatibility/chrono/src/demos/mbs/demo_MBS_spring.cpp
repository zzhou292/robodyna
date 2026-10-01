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
// Authors: Radu Serban
// =============================================================================
//
// Simple example demonstrating the use of ChLinkTSDA.
//
// Two bodies, connected with identical (but modeled differently) spring-dampers
// are created side by side.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef ROBODYNA_CAPTURE_DEMO
    #include "examples/support/CaptureSession.h"
    #include "examples/mbd/Telemetry.h"
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

// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

double rest_length = 1.5;
double spring_coef = 50;
double damping_coef = 1;

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA link.
// In this simple demonstration, we just re-implement the default linear spring-damper.
class MySpringForce : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double restlength,      // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        double force = -spring_coef * (length - restlength) - damping_coef * vel;
        return force;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

#ifdef ROBODYNA_CAPTURE_DEMO
    try {
    const auto capture_options = robodyna::examples::ReadCaptureOptions(
        argc, argv, "src/compatibility/chrono/src/demos/mbs/demo_MBS_spring.cpp", 0.001);
    robodyna::examples::CaptureSession capture(capture_options);
#endif

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body with two visualization spheres
    // -----------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    {
        auto sph_1 = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        ground->AddVisualShape(sph_1, ChFrame<>(ChVector3d(-1, 0, 0), QUNIT));

        auto sph_2 = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        ground->AddVisualShape(sph_2, ChFrame<>(ChVector3d(+1, 0, 0), QUNIT));
    }

    // Create a body suspended through a ChLinkTSDA (default linear)
    // -------------------------------------------------------------

    auto body_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_1);
    body_1->SetPos(ChVector3d(-1, -3, 0));
    body_1->SetFixed(false);
    body_1->EnableCollision(false);
    body_1->SetMass(1);
    body_1->SetInertiaXX(ChVector3d(1, 1, 1));

    // Attach a visualization asset.
    auto box_1 = chrono_types::make_shared<ChVisualShapeBox>(1, 1, 1);
    box_1->SetColor(ChColor(0.6f, 0, 0));
    body_1->AddVisualShape(box_1);

    // Create the spring between body_1 and ground. The spring end points are
    // specified in the body relative frames.
    auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(body_1, ground, true, ChVector3d(0, 0, 0), ChVector3d(-1, 0, 0));
    spring_1->SetRestLength(rest_length);
    spring_1->SetSpringCoefficient(spring_coef);
    spring_1->SetDampingCoefficient(damping_coef);
    sys.AddLink(spring_1);

    // Attach a visualization asset.
    spring_1->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1, 80, 15));

    // Create a body suspended through a ChLinkTSDA (custom force functor)
    // -------------------------------------------------------------------

    auto body_2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_2);
    body_2->SetPos(ChVector3d(1, -3, 0));
    body_2->SetFixed(false);
    body_2->EnableCollision(false);
    body_2->SetMass(1);
    body_2->SetInertiaXX(ChVector3d(1, 1, 1));

    // Attach a visualization asset.
    auto box_2 = chrono_types::make_shared<ChVisualShapeBox>(1, 1, 1);
    box_2->SetColor(ChColor(0, 0, 0.6f));
    body_2->AddVisualShape(box_2);

    // Create the spring between body_2 and ground. The spring end points are
    // specified in the body relative frames.
    auto force = chrono_types::make_shared<MySpringForce>();

    auto spring_2 = chrono_types::make_shared<ChLinkTSDA>();
    spring_2->Initialize(body_2, ground, true, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
    spring_2->SetRestLength(rest_length);
    spring_2->RegisterForceFunctor(force);
    sys.AddLink(spring_2);

    // Attach a visualization asset.
    spring_2->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Create the run-time visualization system
    // ----------------------------------------

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
            vis_irr->SetWindowTitle("ChLinkTSDA demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 0, 6));
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
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("ChLinkTSDA demo");
            vis_vsg->SetBackgroundColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
            vis_vsg->AddCamera(ChVector3d(0, 0, 12));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
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

    // Simulation loop
    double timestep = 0.001;
#ifdef ROBODYNA_CAPTURE_DEMO
    robodyna::examples::mbd::SpringTelemetry telemetry(*body_1, *body_2, rest_length, spring_coef, damping_coef);
    telemetry.Observe(sys.GetChTime(), *body_1, *body_2, *spring_1, *spring_2);
    capture.State(0, sys, capture_options.headless ? nullptr : vis.get());
    for (std::uint64_t step = 1; step <= capture_options.steps; ++step) {
        robodyna::examples::Require(sys.DoStepDynamics(timestep) != 0, "Original spring dynamics step failed");
        telemetry.Observe(sys.GetChTime(), *body_1, *body_2, *spring_1, *spring_2);
        capture.State(step, sys, capture_options.headless ? nullptr : vis.get());
    }
    capture.Finish(sys, telemetry.Summary());
#else
    int frame = 0;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (frame % 50 == 0) {
            std::cout << sys.GetChTime() << "  " << spring_1->GetLength() << "  " << spring_1->GetVelocity() << "  "
                      << spring_1->GetForce() << "\n";

            std::cout << "            " << spring_2->GetLength() << "  " << spring_2->GetVelocity() << "  "
                      << spring_2->GetForce() << "\n\n";
        }

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);

        frame++;
    }
#endif

    return 0;
#ifdef ROBODYNA_CAPTURE_DEMO
    } catch (const std::exception& error) {
        std::cerr << "robodyna original spring: " << error.what() << '\n';
        return 1;
    }
#endif
}
