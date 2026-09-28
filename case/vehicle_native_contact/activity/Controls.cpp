#include "Controls.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_native_contact::activity::values {
namespace s=tlfea::contact::radioss_type25::startup;
native::Controls Self(const RawControls& raw,s::SolidErosion erosion) {
    output::Require(raw.reader_idel==1&&erosion==s::SolidErosion::Enabled,
        "Selected V6 self removal requires authenticated positive I_DEL1 and final solid-erosion policy");
    // Original HM_READ sets IDELKEEP only for negative I_DEL. This source's
    // positive1 therefore keeps the native disconnected-node removal policy.
    return {native::Deletion::ContainingElement,false,erosion};
}
native::Controls Wall(const RawControls& raw,int erosion) {
    output::Require(raw.reader_idel==0&&erosion==0,
        "Declared mesh wall requires native I_DEL0 and disabled solid erosion");
    return {native::Deletion::Disabled,false,s::SolidErosion::Disabled};
}
}
