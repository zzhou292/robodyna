#include "ActualClassification.h"
#include "../Internal.h"
#include "../../tied_search/tests/ActualGeometry.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "lib_utest/qualification/tied_shell_classification/NativeOracle.h"
#include <iostream>

namespace crash::cases::vehicle_startup::classification_actual_test {
const TiedSearchFinalized& Finalized() {
    static const auto value = TiedSearchFinalized::Prepare(TiedSearchAssessment::Prepare(test::OriginalTiedGeometry()));
    return value;
}
const std::string& Member(const char* key, std::size_t cap) {
    // Separate immutable fixture values; no fallback to another input file.
    const auto read = [](const char* variable, std::size_t bytes) {
        const char* path = std::getenv(variable);
        output::Require(path && *path,"Missing explicit original classification fixture");
        return output::ReadBounded(path,bytes);
    };
    static const auto main = read("ROBO_STATIC_MEMBER",42846753);
    static const auto auxiliary = read("ROBO_TIED_AUX_MEMBER",44991);
    static const auto wall = read("ROBO_TIED_WALL_MEMBER",10604);
    output::Require(std::string(key) == "main" || std::string(key) == "auxiliary" || std::string(key) == "wall",
                    "Unknown original classification fixture role");
    const auto& value = std::string(key) == "main" ? main : std::string(key) == "auxiliary" ? auxiliary : wall;
    output::Require(value.size() == cap,"Original classification fixture extent differs");
    return value;
}
const tied::TiedClassificationContext& Context() {
    static const auto value = [] {
        const auto& declaration = Finalized().assessment().geometry().packing().declaration();
        namespace vehicle = modelio::vehicle;
        const auto plan = vehicle::VehicleSourcePlan::ReadBytes(declaration.canonical(),
            vehicle::test::Bytes(),vehicle::test::Identity(vehicle::test::Bytes()));
        const auto rigid = vehicle::rigid_part::RigidPartSource::Prepare(plan,Member("main",42846753));
        const auto auxiliary = tied::TiedAuxiliaryConstraints::Prepare(declaration,Member("auxiliary",44991),
            tied::OriginalWallPolicy::ReplaceWithMeshWall);
        std::cout << "Rigid source startup reservation " << rigid.data().startup_budget_bytes
                  << "; projected source preflight "
                  << tied::TiedClassificationContext::Forecast(auxiliary,rigid,10604) << " bytes\n";
        return tied::TiedClassificationContext::Prepare(auxiliary,rigid,Member("wall",10604),
            tied::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall);
    }();
    return value;
}
const TiedSearchClassification& Prepared() {
    static const auto value = [] {
        const auto forecast = TiedSearchClassification::Forecast(Finalized(),Context());
        std::cout << "Complete classification preflight " << forecast.total_host_bytes << " bytes\n";
        return TiedSearchClassification::Prepare(Finalized(),Context());
    }();
    return value;
}
}
