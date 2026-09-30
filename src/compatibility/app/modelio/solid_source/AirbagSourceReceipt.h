#pragma once

namespace crash::modelio::solid_source::detail {
// Exact block from the original combine.key (lines142..148). Its hash is
// checked against the complete authenticated canonical inventory before use.
// The commented IHQ1 card is retained, never interpreted as an active default.
inline constexpr char AirbagHourglassHash[] =
    "047a527476a6d278a80ee728829df1a7ebffa4c375ccafa46f0b7546d26dd626";
inline constexpr char AirbagHourglassRaw[] =
    "*CONTROL_HOURGLASS\n"
    "$#     ihq        qh\n"
    "$         1       0.1\n"
    "         4      0.02\n"
    "$*******************************************************************************\n"
    "$ Output\n"
    "$*******************************************************************************\n";
} // namespace crash::modelio::solid_source::detail
