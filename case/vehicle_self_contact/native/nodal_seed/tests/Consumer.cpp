#include "case/vehicle_self_contact/native/ContactNodalSeed.h"
int main() {
    using Source = crash::cases::vehicle_self_contact::native::nodal_seed::PreCorrectionNodalSource;
    auto* volatile prepare = &Source::Prepare;
    return prepare ? 0 : 1;
}
