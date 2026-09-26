#include "case/vehicle_self_contact/native/PostGapmMainSource.h"
#include <type_traits>
using Source=crash::cases::vehicle_self_contact::native::post_gapm::PostGapmMainSource;
static_assert(!std::is_default_constructible_v<Source>);
int main(){return 0;}
