#pragma once
#include "lib_utest/qualification/native/t3/T3ForceTestOracle.h"

namespace crash::cases::source_part_elastic::test {
namespace t3_material {
namespace native=tl::qualification::t3;
// Independent continuum LAW1/viscosity/work evaluated in long double from
// the actual constitutive inputs. The separate unchanged kinematics oracle
// checks those inputs against the ideal affine map at its source-scale bounds.
native::force_test::Oracle Independent(const native::ReferenceInput&,const native::HistoryValues&,
    const native::PrescribedInterval&,const native::Kinematics&);
void Check(const native::Reference&,const native::HistoryValues&,const native::PrescribedInterval&,
    const native::ForceTrial&);
} // namespace t3_material
} // namespace crash::cases::source_part_elastic::test
