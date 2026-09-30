#pragma once
#include "case/source_assembly/SourceAssemblyBindings.h"
#include "modelio/source_assembly/tests/AssemblyTestSupport.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <type_traits>

namespace crash::cases::source_assembly::test {
namespace fe=tl::fea;
using source::test::Load;
using source::test::SameBits;
inline SourceAssemblyBindingOptions Options() {
    return {0x5941524953,source::MaterialRatePolicy::OpenRadiossDirectImportDefault};
}
inline std::string Number(double value) { std::ostringstream text; text<<std::setprecision(17)<<value; return text.str(); }
inline void Near(double actual,long double expected) {
    EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),2e-11L*std::max(std::abs(expected),1e-25L));
}
static_assert(!std::is_copy_assignable_v<SourceAssemblyBindings> && !std::is_move_assignable_v<SourceAssemblyBindings>);
static_assert(std::is_nothrow_copy_constructible_v<SourceAssemblyBindings> && std::is_nothrow_move_constructible_v<SourceAssemblyBindings>);
} // namespace crash::cases::source_assembly::test
