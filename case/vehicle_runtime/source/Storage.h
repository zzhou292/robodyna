#pragma once
#include "../Source.h"
#include "../Config.h"
#include "case/vehicle_wall/native/EnvelopeOwnerSource.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include <optional>
#include <variant>
namespace crash::cases::vehicle_runtime {
struct Source::Data {
    struct Original {
        Execution execution;
        Attachments attachments;
        std::optional<vehicle_startup::joints::VehicleJointModel> joints;
    };
    using Environment = vehicle_wall::native::EnvelopeOwnerSource;
    explicit Data(Original source) : value(std::move(source)) {}
    explicit Data(const Environment& source) : value(source) {}
    std::variant<Original,Environment> value;
    const Original* original() const noexcept { return std::get_if<Original>(&value); }
    const Environment* environment() const noexcept { return std::get_if<Environment>(&value); }
};
}
