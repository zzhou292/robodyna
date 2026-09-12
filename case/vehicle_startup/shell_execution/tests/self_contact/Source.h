#pragma once
#include "../../VehicleShellExecution.h"
#include "../../../physical_model/tests/supports/Support.h"
#include "lib_src/collision/SelfContactSurfaceBinding.h"
#include <vector>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace fe = tl::fea;
namespace contact = tlfea::contact;
struct NativeParent {
    double thickness = 0;
    bool centered = false;
    unsigned arity = 0;
    const std::size_t* nodes = nullptr;
};
NativeParent Native(const fe::ShellPhysicalBinding&, const fe::ShellPlasticityParentInput&);
std::uint64_t NativeNodeId(const fe::ShellPhysicalBinding&, const fe::ShellPlasticityParentInput&, unsigned local);
const VehicleShellExecution& Execution();
struct Selection {
    std::vector<contact::SelfContactParentSelection> centered, excluded;
};
const Selection& Inventory();
inline contact::SelfContactSurfaceInput Input(const std::vector<contact::SelfContactParentSelection>& rows) {
    return {rows.data(),rows.size(),contact::SelfContactSurfaceProfile::FrictionlessReferenceThicknessShellSubsetV1};
}
const contact::SelfContactSurfaceBinding& Surface();
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
