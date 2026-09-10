#pragma once
#include "WallRawReport.h"
#include <charconv>
#include <cmath>
#include <string_view>
#include <utility>

// Linux qualification command preflight only. This does not authenticate the
// source/build members of provenance; the external launcher owns that check.
namespace tl::qualification::qeph::wall_recurrence::command {
namespace io=crash::output;
namespace fs=std::filesystem;
inline constexpr std::size_t ExecutableByteCap=64u*1024*1024;
inline bool Hash(std::string_view value) {
  return value.size()==64&&value.find_first_not_of("0123456789abcdef")==std::string_view::npos;
}
inline std::size_t Budget(std::string_view value) {
  io::Require(!value.empty()&&value.size()<=9&&value.front()>='1'&&value.front()<='9'&&
              value.find_first_not_of("0123456789")==std::string_view::npos,
              "Remaining bytes must be a canonical positive decimal integer");
  std::size_t result=0;
  const auto parsed=std::from_chars(value.data(),value.data()+value.size(),result);
  io::Require(parsed.ec==std::errc{}&&parsed.ptr==value.data()+value.size()&&result>0&&
              result<=ScreenSetByteCap,"Remaining bytes exceed the 96 MiB screen-set cap");
  return result;
}
inline std::string Text(const io::Value& object,const char* name) {
  io::Require(object.IsObject()&&object.HasMember(name)&&object[name].IsString(),
              "Missing executable provenance string");
  const auto& value=object[name];
  std::string result(value.GetString(),value.GetStringLength());
  io::Require(!result.empty()&&result.find('\0')==std::string::npos,"Empty or NUL-containing provenance string");
  return result;
}
inline fs::path Path(std::string_view value) {
  io::Require(!value.empty()&&value.size()<=4096,"Missing or oversized path argument");
  return fs::path(value);
}
struct RawInput {
  fs::path directory;
  unsigned cells=0;
  double normal_velocity=0;
  std::string index_sha256,provenance_sha256;
  std::size_t bytes=0;
};
// The analysis and selection commands consume the same six-field raw binding.
// A JSON double preserves negative-zero information that integer parsing loses.
inline RawInput ParseRawInput(const io::Value& raw) {
  io::Require(raw.IsObject()&&raw.MemberCount()==6&&raw.HasMember("directory")&&raw.HasMember("cells")&&
    raw.HasMember("normal_velocity_m_s")&&raw.HasMember("index_sha256")&&
    raw.HasMember("provenance_sha256")&&raw.HasMember("bytes"),"Raw binding requires exactly its declared fields");
  io::Require(raw["cells"].IsUint()&&(raw["cells"].GetUint()==1||raw["cells"].GetUint()==2),
    "Raw cells must be the JSON unsigned integer 1 or 2");
  io::Require(raw["normal_velocity_m_s"].IsDouble(),"Raw normal velocity must be a JSON double -8.0, 0.0 or 8.0");
  const auto velocity=raw["normal_velocity_m_s"].GetDouble();
  io::Require(std::isfinite(velocity)&&FrozenVelocity(velocity)&&!(velocity==0&&std::signbit(velocity)),
    "Raw normal velocity is outside the frozen tuple or is negative zero");
  io::Require(raw["bytes"].IsUint64()&&raw["bytes"].GetUint64()>0&&raw["bytes"].GetUint64()<=ScreenSetByteCap,
    "Raw bytes must be a positive bounded JSON unsigned integer");
  RawInput result; result.cells=raw["cells"].GetUint(); result.normal_velocity=velocity;
  result.index_sha256=Text(raw,"index_sha256"); result.provenance_sha256=Text(raw,"provenance_sha256");
  io::Require(Hash(result.index_sha256)&&Hash(result.provenance_sha256),"Raw hashes must be 64 lowercase hexadecimal characters");
  result.bytes=static_cast<std::size_t>(raw["bytes"].GetUint64());
  const auto path=Path(Text(raw,"directory"));
  io::Require(fs::is_directory(fs::symlink_status(path)),"Raw input must be a real directory");
  result.directory=fs::canonical(path); return result;
}
inline void NewDirectory(const fs::path& destination) {
  io::Require(!fs::exists(fs::symlink_status(destination)),"Output directory must be new, including absent symlinks");
  const auto parent=destination.has_parent_path()?destination.parent_path():fs::path(".");
  io::Require(fs::is_directory(parent),"Output parent directory must exist");
}
struct Provenance { std::string bytes; io::Document parsed; };
inline Provenance ReadProvenance(const fs::path& path,std::string_view expected_hash,std::size_t remaining) {
  auto bytes=io::ReadBounded(path,ProvenanceByteCap);
  io::Require(io::Sha256(bytes)==expected_hash,"Exact provenance SHA256 mismatch");
  auto parsed=ParseRawProvenance(bytes);
  io::Require(bytes.size()<remaining,"Provenance exhausts remaining screen byte budget");
  return {std::move(bytes),std::move(parsed)};
}
struct Executable { fs::path path; std::string sha256; };
inline Executable CheckExecutable(const io::Document& parsed) {
  io::Require(parsed.HasMember("executable")&&parsed["executable"].IsObject(),"Missing executable provenance object");
  const auto expected_executable=Path(Text(parsed["executable"],"path"));
  const auto expected_executable_hash=Text(parsed["executable"],"sha256");
  io::Require(Hash(expected_executable_hash),"Executable SHA256 must be 64 lowercase hexadecimal characters");
  const auto executable=fs::canonical("/proc/self/exe");
  io::Require(fs::canonical(expected_executable)==executable,"Running executable path differs from provenance");
  // Read the running image, not a replacement file at its pathname.
  const auto executable_hash=io::Sha256(io::ReadBounded("/proc/self/exe",ExecutableByteCap));
  io::Require(executable_hash==expected_executable_hash,"Running executable SHA256 differs from provenance");
  return {executable,executable_hash};
}
} // namespace tl::qualification::qeph::wall_recurrence::command
