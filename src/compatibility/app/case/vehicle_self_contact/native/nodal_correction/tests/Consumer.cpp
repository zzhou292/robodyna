#include "case/vehicle_self_contact/native/CorrectedNodalSource.h"
int main() {
    using Source = crash::cases::vehicle_self_contact::native::nodal_correction::CorrectedNodalSource;
    auto* volatile prepare = &Source::Prepare;
    auto volatile slots = &Source::material_slots;
    return prepare && slots ? 0 : 1;
}
