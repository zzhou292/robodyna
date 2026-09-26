#pragma once
#include "../CoefficientValues.h"
#include "lib_utest/qualification/radioss_type25_reader_solid_coefficients/NativeOracle.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
detail::MainResult Native(const detail::MainPacket&, const std::array<double,24>& second_coordinates);
}
