#pragma once

#include "chrono_fmi/fmi2/ChFmuForgeImport.h"
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace robodyna::test {
inline void RequireFmi(fmi2Status status, const std::string& operation) {
    if (status != fmi2OK) throw std::runtime_error(operation + ": FMI status " + std::to_string(status));
}

// One real native FMI instance. The inherited importer does not unload DSOs;
// this bounded gate explicitly releases component state and makes no long-run
// importer-unload/lifecycle claim.
class FmuInstance {
  public:
    FmuInstance(const std::string& archive, const std::string& name) {
        const char* temporary = std::getenv("TEST_TMPDIR");
        if (!temporary) throw std::runtime_error("Missing test output root");
        const auto unpack = std::filesystem::path(temporary) / name;
        if (!std::filesystem::create_directory(unpack)) throw std::runtime_error("FMU output must be new");
        unit.Load(fmi2CoSimulation, archive, unpack.string());
        unit.Instantiate(name, false, false);  // real visual FMU, no window requested
        RequireFmi(unit.SetupExperiment(fmi2False, 0, 0, fmi2False, 1), "SetupExperiment");
    }
    ~FmuInstance() {
        if (unit.component) {
            if (initialized) unit._fmi2Terminate(unit.component);
            unit._fmi2FreeInstance(unit.component);
            unit.component = nullptr;
        }
    }
    FmuInstance(const FmuInstance&) = delete;
    FmuInstance& operator=(const FmuInstance&) = delete;

    void Initialize() {
        RequireFmi(unit.EnterInitializationMode(), "EnterInitializationMode");
        RequireFmi(unit.ExitInitializationMode(), "ExitInitializationMode");
        initialized = true;
    }
    void Set(const std::string& name, double value) {
        RequireFmi(unit.SetVariable(name, value, chrono::fmi2::FmuVariable::Type::Real), "Set " + name);
    }
    double Get(const std::string& name) {
        double value = 0;
        RequireFmi(unit.GetVariable(name, value, chrono::fmi2::FmuVariable::Type::Real), "Get " + name);
        return value;
    }
    void SetVector(const std::string& name, const chrono::ChVector3d& value) {
        RequireFmi(unit.SetVecVariable(name, value), "Set vector " + name);
    }
    chrono::ChVector3d GetVector(const std::string& name) {
        chrono::ChVector3d value;
        RequireFmi(unit.GetVecVariable(name, value), "Get vector " + name);
        return value;
    }
    chrono::ChFrameMoving<> Frame() {
        chrono::ChFrameMoving<> value;
        RequireFmi(unit.GetFrameMovingVariable("ref_frame", value), "Get vehicle frame");
        return value;
    }
    void Step(double time, double dt) { RequireFmi(unit.DoStep(time, dt, fmi2True), "DoStep"); }

    chrono::fmi2::FmuChronoUnit unit;

  private:
    bool initialized = false;
};
}  // namespace robodyna::test
