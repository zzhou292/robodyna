#pragma once
#include "WallRawReport.h"
#include <charconv>
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
