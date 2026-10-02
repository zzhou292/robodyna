#pragma once

#include "chrono/serialization/ChArchive.h"

#include <memory>
#include <utility>
#include <vector>

namespace robodyna::fmi {

// An archive visits values synchronously; an FMU keeps their bindings afterward.
// Copy explicitly constant metadata so stack-local archive versions/dimensions
// remain valid. Ordinary model members retain their original live references.
class ArchiveValueBindings {
  public:
    template <class T>
    chrono::ChNameValue<T> CaptureConstant(chrono::ChNameValue<T> value) {
        if (value.GetVariability() != chrono::ChVariabilityType::constant)
            return value;
        auto owned = std::make_shared<T>(value.const_value());
        chrono::ChNameValue<T> result(value.name(), *owned, value.flags(),
                                     value.GetCausality(), value.GetVariability());
        constants_.push_back(std::move(owned));
        return result;
    }

  private:
    std::vector<std::shared_ptr<void>> constants_;
};

// FMI2 permits continuous variability only for Real. Integer/Boolean/String
// model members remain live but are discrete; explicit metadata is preserved.
inline chrono::ChVariabilityType ScalarArchiveVariability(bool is_real, chrono::ChVariabilityType value) {
    return !is_real && value == chrono::ChVariabilityType::continuous
               ? chrono::ChVariabilityType::discrete
               : value;
}

}  // namespace robodyna::fmi
