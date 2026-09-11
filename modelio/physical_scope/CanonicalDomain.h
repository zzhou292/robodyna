#pragma once
#include "PhysicalScope.h"
#include "lib_src/assembly/NodalNodeDomain.h"

namespace crash::modelio::physical_scope {
// Every baseline/TYPE25 source node is required; a declared superset is allowed
// only when each added NID and coordinate matches the original canonical bits.
// This checks source geometry, not coefficients, DOFs or mechanical closure.
void ValidateDeclaredDomain(const PhysicalScope&, const tl::fea::NodalNodeDomain&);
namespace detail {
void CheckDomain(const std::vector<std::uint64_t>&, const std::vector<double>&,
                 const std::vector<std::uint16_t>&, const tl::fea::NodalNodeDomain&,
                 std::uint64_t source_instance, std::size_t required_count);
}
} // namespace crash::modelio::physical_scope
