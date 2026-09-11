#pragma once
#include "../VehiclePhysicalModel.h"
#include "modelio/physical_scope/tests/ActualSupport.h"
#include "modelio/vehicle_sections/tests/RigidSourceSupport.h"
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>

namespace crash::cases::vehicle_startup::physical_model::test {
namespace fe = tl::fea;
inline const modelio::physical_domain::VehiclePhysicalDomain& Source() {
    static const auto value = modelio::physical_domain::VehiclePhysicalDomain::Prepare(
        modelio::physical_scope::test::Actual(), modelio::physical_domain::Policy::RetainedShellAssembliesV1);
    return value;
}
inline const VehicleShellBinding& Shells() {
    static const auto value = VehicleShellBinding::Prepare(
        VehicleShellReferences::Prepare(modelio::vehicle::test::RigidResolution()));
    return value;
}
inline const VehiclePhysicalModel& Actual() {
    static const auto value = [] {
        std::cout << "VehiclePhysicalModel complete forecast=" <<
            VehiclePhysicalModel::Preflight(Source(), Shells()).total_bytes << std::endl;
        return VehiclePhysicalModel::Prepare(Source(), Shells());
    }();
    return value;
}
inline void Same(double a, double b) { EXPECT_EQ(output::Bits(a), output::Bits(b)); }
template<class T> std::string RecordValue(T value) {
    std::ostringstream text; text << std::setprecision(17) << value; return text.str();
}
} // namespace crash::cases::vehicle_startup::physical_model::test
