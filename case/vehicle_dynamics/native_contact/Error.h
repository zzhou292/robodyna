#pragma once
#include "Observation.h"
#include <stdexcept>

namespace crash::cases::vehicle_dynamics::native_contact {
enum class Operation { Adopt, Assemble, SealCandidate };
struct Failure {
    Operation operation = Operation::Adopt;
    native::TransactionStatus status = native::TransactionStatus::InvalidInput;
    std::size_t row = SIZE_MAX, occurrence = SIZE_MAX;
    native::selection::Status selection_status = native::selection::Status::Ok;
};
class StageError : public std::runtime_error {
  public:
    StageError(Operation, const native::TransactionReport&);
    const Failure& failure() const noexcept { return failure_; }
  private:
    Failure failure_;
};
} // namespace crash::cases::vehicle_dynamics::native_contact
