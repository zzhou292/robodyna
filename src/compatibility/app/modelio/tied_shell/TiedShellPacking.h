#pragma once
#include "TiedShellDeclaration.h"

namespace crash::modelio::tied_shell {
inline constexpr const char* PackingPolicy =
    "openradioss_a62b27e_single_part_clause_unique_shell_tuple_v1";
inline constexpr const char* PackingRevision =
    "a62b27e6baa555d222a580d6218867d0be4d70b5";

struct PackingLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t masters = 524288;
    std::size_t nodes = 1048576;
};
struct PackingData {
    // Each value indexes declaration.data().masters; the inverse maps that
    // declaration row to native-compatible IRECT rank. Neither is a native EID.
    std::vector<std::uint32_t> master_rows, master_ranks;
    std::size_t startup_budget_bytes = 0, owned_payload_bytes = 0;
};

// Packing only. The retained declaration's source-ordered constraint evidence,
// classification and search remain unchanged and unresolved. Slave/master-node
// order is the declaration's existing ascending-NID order, with original NIDs
// mapped monotonically to native internal indices by the selected import path.
class TiedShellPacking {
  public:
    static TiedShellPacking Prepare(const TiedShellDeclaration&, PackingLimits = {});
    static std::size_t Forecast(const TiedShellDeclaration&, PackingLimits = {});
    TiedShellPacking(const TiedShellPacking&) noexcept = default;
    TiedShellPacking(TiedShellPacking&& other) noexcept : storage_(other.storage_) {}
    TiedShellPacking& operator=(const TiedShellPacking&) = delete;
    TiedShellPacking& operator=(TiedShellPacking&&) = delete;
    const TiedShellDeclaration& declaration() const noexcept;
    const PackingData& data() const noexcept;
  private:
    struct Storage;
    explicit TiedShellPacking(std::shared_ptr<const Storage> storage)
        : storage_(std::move(storage)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::tied_shell
