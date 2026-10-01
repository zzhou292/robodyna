#pragma once

#include <string>

#include "tests/serialization_compat/FixtureModel.h"

namespace robodyna::serialization_compat {
enum class Format { Json, Xml, Binary };
const char* Extension(Format format);
std::string WriteFixture(Format format, FixtureModel& fixture);
void ReadFixture(Format format, const std::string& bytes, FixtureModel& fixture);
std::string ReadBytes(const std::string& path);
}  // namespace robodyna::serialization_compat
