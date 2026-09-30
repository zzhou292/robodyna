#include "case/vehicle_self_contact/native/SelectedShellMainSource.h"
#include <type_traits>
int main() {
    using S = crash::cases::vehicle_self_contact::native::main_coefficients::SelectedShellMainSource;
    static_assert(std::is_copy_constructible_v<S> && !std::is_default_constructible_v<S>);
    return 0;
}
