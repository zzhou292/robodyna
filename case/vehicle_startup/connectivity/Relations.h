#pragma once
#include "Types.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
class Relations {
  public:
    Relations(Data&, const Counts&);
    void Append(Kind, Role, std::uint64_t id, std::uint64_t part, std::size_t source_row,
                const std::size_t* slots, std::size_t count, std::size_t rigid_root = SIZE_MAX);
    void Complete() const;
  private:
    Data& data_;
    Counts expected_;
};
} // namespace crash::cases::vehicle_startup::connectivity::detail
