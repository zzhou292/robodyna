#pragma once

#include "chrono_thirdparty/rapidjson/document.h"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace crash::output {

using Document = rapidjson::Document;
using Value = rapidjson::Value;

// Shared artifact primitives, extracted from the normal-impact writer. All
// failures throw; callers own run failure reporting and successful inventories.
// File writers require absent destinations and check flush/close completion.
// Calls/filesystem ownership are externally serialized; these are not atomic
// exclusive-create or multi-file transaction operations.
void Require(bool condition, const char* message);
std::string ReadBounded(const std::filesystem::path& path, std::size_t byte_cap);
std::string Sha256(const std::string& bytes);
std::uint64_t Bits(double value);

void String(Document&, const char* name, const std::string& value);
void Number(Document&, const char* name, double value);
void Integer(Document&, const char* name, std::uint64_t value);
void Boolean(Document&, const char* name, bool value);
void WriteJson(const std::filesystem::path& path, const Document&);
void WriteBytes(const std::filesystem::path& path, const std::string& bytes);

}  // namespace crash::output
