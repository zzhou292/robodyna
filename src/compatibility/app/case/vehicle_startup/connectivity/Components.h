#pragma once
#include "Types.h"
#include "lib_src/assembly/NodalNodeDomain.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
// Temporary bounded union-find over an already authenticated node domain.
// Minimum source NID labels do not depend on relation order or forest roots.
class Components {
  public:
    explicit Components(tl::util::ConstView<tl::fea::NodalDomainNode>);
    void Join(const std::uint32_t* slots, std::size_t count);
    std::size_t Labels(std::vector<std::uint64_t>&);
  private:
    std::uint32_t Root(std::uint32_t);
    tl::util::ConstView<tl::fea::NodalDomainNode> nodes_;
    std::vector<std::uint32_t> parent_;
};
// Complete validation precedes the first union or publication to output.
void Partition(tl::util::ConstView<tl::fea::NodalDomainNode>, Data&);
} // namespace crash::cases::vehicle_startup::connectivity::detail
