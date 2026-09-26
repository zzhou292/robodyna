#include "case/vehicle_wall/native/FiniteWallContactSource.h"
int main() {
    namespace source=crash::cases::vehicle_wall::native::wall_interface;
    using Factory=source::Preparation(*)(const crash::cases::vehicle_wall::native::EnvelopeOwnerSource&,
        const source::VehicleSource&,source::Declaration,source::Limits);
    Factory volatile function=&source::FiniteWallContactSource::Prepare;
    return function?0:1;
}
