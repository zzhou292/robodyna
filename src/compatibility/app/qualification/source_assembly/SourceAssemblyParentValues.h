#pragma once
#include "SourceAssemblyFlightFixture.h"
namespace crash::qualification::source_assembly {
// Existing field-complete qualification reductions shared by both source laws.
void CheckTriangleValues(const t::ReferenceData&,const t::PrescribedInterval&,const t::ForceTrial&,const t::ForceTrial&);
void CheckPlasticSectionValues(const fe::ShellBatchSectionState&,const fe::sections::ShellLayeredJ2History&,
    const fe::sections::ShellLayeredJ2Diagnostics&,double expected_work);
}
