#pragma once
#include "lib_src/elements/type13/Type13Startup.h"

namespace crash::modelio::type13 {
namespace native=tl::fea::type13;
// Explicit source working values after original-card/policy validation.
struct SourceProperty {
    double density=0,young=0,poisson=0,yield=0,tangent=0;
    double outer1=0,outer2=0,inner1=0,inner2=0;
    double failure_deformation=0;
};
// Owning adapter buffer; input() rebinds its curve pointers on each call.
struct ConvertedProperty {
    native::Curve curves[4]{};
    native::ChannelInput channels[6]{};
    double mass_per_length=0,inertia_per_length=0;
    native::PropertyInput input() const noexcept;
};
// Exact selected ConvertSectionBeamToSpringBeam expression order. No solver
// defaults, recurrence or source keywords enter TL's material implementation.
native::Status ConvertProperty(const SourceProperty&,ConvertedProperty&);
}
