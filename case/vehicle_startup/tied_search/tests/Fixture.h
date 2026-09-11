#pragma once
#include "../Internal.h"
#include "modelio/tied_shell/tests/TinyFixture.h"
#include "modelio/tied_shell/packing/Internal.h"
#include "modelio/tied_shell/search_geometry/Internal.h"
namespace crash::cases::vehicle_startup::test {
namespace detail = tied_assessment_detail;
struct Fixture : tied::test::TinyFixture {
    tied::Data declaration;
    tied::PackingData packing;
    tied::SearchGeometryData geometry;
    Fixture() : TinyFixture(false, true), declaration(Prepare()),
        packing(tied::packing_detail::Build(canonical, declaration, {})),
        geometry(tied::search_detail::Build(canonical, declaration, packing, member, {})) {}
};
}
