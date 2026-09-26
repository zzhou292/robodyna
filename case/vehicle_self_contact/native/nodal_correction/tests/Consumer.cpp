#include "case/vehicle_self_contact/native/CorrectedNodalSource.h"
int main() {
    using Source = crash::cases::vehicle_self_contact::native::nodal_correction::CorrectedNodalSource;
    auto* volatile prepare = &Source::Prepare;
    return prepare ? 0 : 1;
}
