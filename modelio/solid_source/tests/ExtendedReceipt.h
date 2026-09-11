#pragma once
#include <cstddef>
#include <cstdint>

namespace crash::modelio::solid_source::test {
// Independently authenticated source audit, not a production selector.
// yaris-remaining-metal-rubber-source-1.json SHA256
// 751a2834dd7c31d9fe84633589ab6c17ce430d7417b43beb305f121038cbb2e5.
struct RubberReceipt {
    std::uint64_t pid;
    std::size_t bricks, wedges;
    double source_density;
    const char* ordered_records_sha256;
};
inline constexpr RubberReceipt AddedRubber[]{
    {2000017, 226, 71, 1.98e-9, "66d6f8eb5766b199ab5d9608f7121f953c7f40d813ba07f853ed749f0f2bdc71"},
    {2000393, 72, 0, 1.98e-9, "4d10cec21faf8dd42a4b6ce6a7656c4011b2fbb837e829ec273a2489ea8536bb"},
    {2000509, 192, 42, 1.999e-9, "c594684304b8e062d86ed07be6c7ec7744bb304a981aaff29060bf0187dff333"},
    {2000521, 192, 42, 1.999e-9, "5f1896c3c7173a6482df1e5a41b4ace8a42f07044d72dc00c36d7beb814f04c3"}
};
inline const RubberReceipt* Added(std::uint64_t pid) noexcept {
    for (const auto& part : AddedRubber)
        if (part.pid == pid) return &part;
    return nullptr;
}
} // namespace crash::modelio::solid_source::test
