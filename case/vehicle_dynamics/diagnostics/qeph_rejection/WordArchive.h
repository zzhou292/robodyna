#pragma once
#include "Codec.h"
#include "output/ArtifactIO.h"
#include <cstring>
#include <limits>
#include <type_traits>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::codec {
class Writer {
  public:
    static constexpr bool Reading = false;
    std::vector<std::uint64_t> words;
    Writer() { words.reserve(MaximumWords); }
    template<class T> void operator()(const T& value) {
        if constexpr (std::is_array_v<T>) {
            for (const auto& item : value) (*this)(item);
        } else if constexpr (std::is_enum_v<T>) {
            (*this)(static_cast<std::underlying_type_t<T>>(value));
        } else {
            static_assert(std::is_integral_v<T> || std::is_same_v<T, double>);
            std::uint64_t word = 0;
            if constexpr (std::is_same_v<T, double>) std::memcpy(&word, &value, sizeof(word));
            else if constexpr (std::is_signed_v<T>) word = static_cast<std::uint64_t>(static_cast<std::int64_t>(value));
            else word = static_cast<std::uint64_t>(value);
            output::Require(words.size() < MaximumWords, "QEPH diagnostic encoding exceeds its word cap");
            words.push_back(word);
        }
    }
    template<class First, class... Rest> void operator()(const First& first, const Rest&... rest) {
        (*this)(first); ((*this)(rest), ...);
    }
};
class Reader {
  public:
    static constexpr bool Reading = true;
    explicit Reader(const std::vector<std::uint64_t>& values) : words(values) {
        output::Require(values.size() <= MaximumWords, "QEPH diagnostic input exceeds its word cap");
    }
    template<class T> void operator()(T& value) {
        if constexpr (std::is_array_v<T>) {
            for (auto& item : value) (*this)(item);
        } else if constexpr (std::is_enum_v<T>) {
            std::underlying_type_t<T> raw{}; (*this)(raw); value = static_cast<T>(raw);
        } else {
            static_assert(std::is_integral_v<T> || std::is_same_v<T, double>);
            output::Require(cursor < words.size(), "QEPH diagnostic input is truncated");
            const auto word = words[cursor++];
            if constexpr (std::is_same_v<T, double>) std::memcpy(&value, &word, sizeof(word));
            else if constexpr (std::is_signed_v<T>) {
                std::int64_t signed_value; std::memcpy(&signed_value, &word, sizeof(word));
                output::Require(signed_value >= std::numeric_limits<T>::min() &&
                    signed_value <= std::numeric_limits<T>::max(), "QEPH diagnostic integer is out of range");
                value = static_cast<T>(signed_value);
            } else {
                output::Require(word <= std::numeric_limits<T>::max(), "QEPH diagnostic integer is out of range");
                value = static_cast<T>(word);
            }
        }
    }
    template<class First, class... Rest> void operator()(First& first, Rest&... rest) {
        (*this)(first); ((*this)(rest), ...);
    }
    void Finish() const { output::Require(cursor == words.size(), "QEPH diagnostic input has trailing words"); }
  private:
    const std::vector<std::uint64_t>& words;
    std::size_t cursor = 0;
};
}
