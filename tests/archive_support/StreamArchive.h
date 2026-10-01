// Shared archive stream lifetime helpers for isolated compatibility fixtures.
#pragma once
#include <sstream>
#include <stdexcept>
#include <string>
#include "chrono/serialization/ChArchive.h"

namespace robodyna::archive_test {
using chrono::make_ChNameValue;

template <class Archive, class Fixture>
std::string Write(Fixture& fixture) {
    std::ostringstream stream(std::ios::out | std::ios::binary);
    { Archive archive(stream); archive << CHNVP(fixture); }
    if (!stream)
        throw std::runtime_error("fixture archive write failed");
    return stream.str();
}

template <class Archive, class Fixture>
void Read(const std::string& bytes, Fixture& fixture) {
    std::istringstream stream(bytes, std::ios::in | std::ios::binary);
    { Archive archive(stream); archive >> CHNVP(fixture); }
}
}  // namespace robodyna::archive_test
