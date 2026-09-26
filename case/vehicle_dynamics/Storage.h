#pragma once
#include "VehiclePhysicalDynamics.h"
#include "ExecutionAccess.h"
#include "SelfContactContribution.h"
#include "WallContribution.h"
#include "native_contact/Group.h"
#include "../vehicle_startup/TiedCinWitnessActivity.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <array>
namespace crash::cases::vehicle_dynamics {
struct VehiclePhysicalDynamics::Storage {
    Storage(vehicle_runtime::VehiclePhysicalStartup&& value,Config c,Forecast f);
    vehicle_runtime::VehiclePhysicalStartup startup;
    Config config;
    Forecast forecast;
    StepTimer timer;
    std::unique_ptr<vehicle_startup::TiedCinWitnessActivity> activity;
    std::unique_ptr<detail::WallContribution> wall;
    std::unique_ptr<detail::SelfContactContribution> self_contact;
    std::unique_ptr<native_contact::Group> native_contact;
    tl::fea::NodalUniformMotionObserver motion;
    std::array<StepObservation,2> observations;
    tl::fea::NodalTrialToken token;
    tl::fea::NodalPreparedView prepared;
    tl::fea::NodalStamp stamp;
    unsigned accepted_slot=0;
    bool pending=false;
    ExecutionAccess::State& state() noexcept { return ExecutionAccess::Get(startup); }
    StepObservation& candidate() noexcept { return observations[1-accepted_slot]; }
    void Prepare();
    void Evaluate();
    void Capture();
    void Discard() noexcept;
};
} // namespace crash::cases::vehicle_dynamics
