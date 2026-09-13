#pragma once
#include "../../VehicleShellExecution.h"
#include "../../../physical_model/tests/supports/Support.h"
#include "lib_src/collision/SelfContactSurfaceBinding.h"
#include "modelio/self_contact/OriginalSelection.h"
#include "case/vehicle_self_contact/VehicleSelfContactSetup.h"
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
const vehicle_runtime::Attachments& PhysicalAttachments();
struct Selection {
    std::vector<contact::SelfContactParentSelection> centered, excluded;
};
const Selection& Inventory();
const modelio::self_contact::OriginalSelection& OriginalContactSelection();
const Selection& ContactInventory();
inline contact::SelfContactSurfaceInput Input(const std::vector<contact::SelfContactParentSelection>& rows) {
    return {rows.data(),rows.size(),contact::SelfContactSurfaceProfile::FrictionlessReferenceThicknessShellSubsetV1};
}
const contact::SelfContactSurfaceBinding& Surface();
const contact::SelfContactSurfaceBinding& ContactSurface();
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
