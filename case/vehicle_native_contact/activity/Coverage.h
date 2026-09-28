#pragma once
#include "Declaration.h"
#include "output/BoundedArrayIO.h"
namespace crash::cases::vehicle_native_contact::activity::coverage {
using Canonical=output::full_shell::source::CanonicalData;
Coverage Audit(const detail::SourceInputs&,Limits);
Population Shells(const detail::SourceInputs&,const Canonical&);
Population Solids(const detail::SourceInputs&,const Canonical&);
Population Beams(const detail::SourceInputs&,const Canonical&);
void Connections(const detail::SourceInputs&,Coverage&);
// Reusable exact source-row checks used by all array-backed families.
void CheckRecord(const std::vector<std::uint64_t>&,std::size_t columns,std::size_t row,
    std::uint64_t id,const std::uint64_t* nodes,unsigned arity,std::vector<std::uint8_t>& selected);
Population Finish(const std::vector<std::uint64_t>&,std::size_t columns,unsigned arity,
    const std::vector<std::uint8_t>&,const tl::fea::NodalNodeDomain&);
}
