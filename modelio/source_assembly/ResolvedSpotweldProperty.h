#pragma once
#include "lib_src/elements/type25/Type25Types.h"

namespace crash::modelio::assembly {
inline constexpr tl::fea::type25::SourceUnits SpotweldSourceUnits{1000., .001, 1.};
// Pinned direct-import blank WID/N1/N2-only property in original t/mm/s units.
// The source adapter must admit that exact card policy before using this value.
tl::fea::type25::Property ResolvedSpotweldProperty();
}
