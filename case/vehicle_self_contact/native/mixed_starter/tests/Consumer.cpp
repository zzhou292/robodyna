#include "case/vehicle_self_contact/native/MixedStarterSource.h"
int main() {
    namespace source=crash::cases::vehicle_self_contact::native::mixed_starter;
    using Factory=source::Preparation(*)(const source::MainSource&,source::Limits);
    Factory volatile function=static_cast<Factory>(&source::MixedStarterSource::Prepare);
    return function?0:1;
}
