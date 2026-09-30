#include "case/vehicle_self_contact/native/ContactGapOperands.h"
#include <type_traits>
int main() {
    using Source=crash::cases::vehicle_self_contact::native::gap_operands::ContactGapOperands;
    static_assert(std::is_copy_constructible_v<Source> && !std::is_default_constructible_v<Source>);
    return 0;
}
