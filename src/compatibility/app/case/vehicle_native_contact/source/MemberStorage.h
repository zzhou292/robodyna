#pragma once
#include "output/ArtifactIO.h"
#include <algorithm>
#include <string>
namespace crash::cases::vehicle_native_contact::source::detail {
// ReadBounded retains its qualified 4096-byte append reader. The pinned
// libstdc++ doubles string capacity; charge both growth allocations and the
// later compact copy before reading. Validate the actual returned capacity.
inline std::size_t ReaderCapacity(std::size_t maximum_bytes) {
    const auto n=std::max(maximum_bytes,std::size_t{4096});
    output::Require(n<=(SIZE_MAX-1)/2,"Original member reader reservation overflows");
    return 2*n+1;
}
inline std::size_t ReadAndCompactPeak(std::size_t owned_cap,std::size_t maximum_bytes) {
    const auto reader=ReaderCapacity(maximum_bytes);
    output::Require(maximum_bytes<=SIZE_MAX-4096-64&&reader<=(SIZE_MAX-owned_cap)/2,
        "Original member compaction reservation overflows");
    const auto base=owned_cap+2*reader;
    output::Require(maximum_bytes+4096+64<=SIZE_MAX-base,"Original member compaction reservation overflows");
    return base+maximum_bytes+4096+64;
}
inline std::string CompactMember(std::string value,std::size_t maximum_bytes) {
    output::Require(value.size()<=maximum_bytes&&value.capacity()<ReaderCapacity(maximum_bytes),
        "Original member reader capacity exceeds reserved growth");
    // Explicit copy is used instead of nonbinding shrink_to_fit. This changes
    // storage only; authentication and all original bytes remain unchanged.
    std::string compact(value.data(),value.size());
    output::Require(compact.capacity()<=value.size()+63,
        "Compact original member capacity exceeds its reservation");
    return compact;
}
} // namespace crash::cases::vehicle_native_contact::source::detail
