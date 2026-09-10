#include "WallRawReport.h"
#include <charconv>
#include <iostream>
#include <string_view>

namespace wall=tl::qualification::qeph::wall_recurrence;
namespace io=crash::output;
namespace fs=std::filesystem;
namespace {
constexpr std::size_t ExecutableByteCap=64u*1024*1024;
bool Hash(std::string_view value) {
  return value.size()==64&&value.find_first_not_of("0123456789abcdef")==std::string_view::npos;
}
std::size_t Budget(std::string_view value) {
  io::Require(!value.empty()&&value.size()<=9&&value.front()>='1'&&value.front()<='9'&&
              value.find_first_not_of("0123456789")==std::string_view::npos,
              "Remaining bytes must be a canonical positive decimal integer");
  std::size_t result=0;
  const auto parsed=std::from_chars(value.data(),value.data()+value.size(),result);
  io::Require(parsed.ec==std::errc{}&&parsed.ptr==value.data()+value.size()&&result>0&&
              result<=wall::ScreenSetByteCap,"Remaining bytes exceed the 96 MiB screen-set cap");
  return result;
}
std::string Text(const io::Value& object,const char* name) {
  io::Require(object.IsObject()&&object.HasMember(name)&&object[name].IsString(),
              "Missing executable provenance string");
  const auto& value=object[name];
  std::string result(value.GetString(),value.GetStringLength());
  io::Require(!result.empty()&&result.find('\0')==std::string::npos,"Empty or NUL-containing provenance string");
  return result;
}
fs::path Path(std::string_view value) {
  io::Require(!value.empty()&&value.size()<=4096,"Missing or oversized path argument");
  return fs::path(value);
}
}

// Linux qualification CLI only. Exact provenance is retained unchanged;
// executable identity is checked here, source/build input authentication stays
// with the root launcher. All argument/input checks precede directory creation.
// Collection completion is independent of local checks or spectral admission.
int main(int argc,char** argv) {
  try {
    io::Require(argc==7,"Usage: qeph_wall_raw CELLS BOOST PROVENANCE_FILE EXPECTED_PROVENANCE_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES");
    const std::string_view cells_arg=argv[1],boost_arg=argv[2],expected_hash=argv[4];
    io::Require(cells_arg=="1"||cells_arg=="2","CELLS must be exactly 1 or 2");
    io::Require(boost_arg=="-8"||boost_arg=="0"||boost_arg=="8","BOOST must be exactly -8, 0 or 8");
    io::Require(Hash(expected_hash),"Expected provenance SHA256 must be 64 lowercase hexadecimal characters");
    const unsigned cells=static_cast<unsigned>(cells_arg.front()-'0');
    const double boost=boost_arg=="-8"?-8.:(boost_arg=="8"?8.:0.);
    const auto remaining=Budget(argv[6]);
    const auto provenance_path=fs::canonical(Path(argv[3]));
    const auto destination=Path(argv[5]);
    io::Require(fs::is_regular_file(provenance_path),"Provenance must be a regular file");
    io::Require(!fs::exists(fs::symlink_status(destination)),"Output directory must be new, including absent symlinks");
    const auto parent=destination.has_parent_path()?destination.parent_path():fs::path(".");
    io::Require(fs::is_directory(parent),"Output parent directory must exist");
    const auto provenance=io::ReadBounded(provenance_path,wall::ProvenanceByteCap);
    io::Require(io::Sha256(provenance)==expected_hash,"Exact provenance SHA256 mismatch");
    const auto parsed=wall::ParseRawProvenance(provenance);
    io::Require(provenance.size()<remaining,"Provenance exhausts remaining screen byte budget");
    io::Require(parsed.HasMember("executable")&&parsed["executable"].IsObject(),"Missing executable provenance object");
    const auto expected_executable=Path(Text(parsed["executable"],"path"));
    const auto expected_executable_hash=Text(parsed["executable"],"sha256");
    io::Require(Hash(expected_executable_hash),"Executable SHA256 must be 64 lowercase hexadecimal characters");
    const auto executable=fs::canonical("/proc/self/exe");
    io::Require(fs::canonical(expected_executable)==executable,"Running executable path differs from provenance");
    // Read the running image, not a replacement file at its pathname.
    const auto executable_hash=io::Sha256(io::ReadBounded("/proc/self/exe",ExecutableByteCap));
    io::Require(executable_hash==expected_executable_hash,"Running executable SHA256 differs from provenance");
    std::cout<<"Observed executable: "<<executable<<" sha256="<<executable_hash
             <<"; provenance_sha256="<<expected_hash<<'\n'<<std::flush;
    wall::RawJobWriter writer(destination,cells,boost,provenance,provenance_path.string(),remaining);
    const auto job=wall::CollectRawJob(cells,boost,[&writer](const wall::RawJob& state,wall::RawProgress event) {
      writer(state,event);
    });
    const auto& receipt=writer.receipt();
    io::Require(receipt.final_index_present&&receipt.collection_complete==job.collection_complete,
                "Raw collector and retained final index disagree");
    std::cout<<"Retained "<<receipt.files.size()<<" files, "<<receipt.total_bytes
             <<" bytes; raw_collection_complete="<<job.collection_complete
             <<"; screen_admitted=false; trajectory_admitted=false\n";
    return job.collection_complete?0:2;
  } catch(const std::exception& error) {
    std::cerr<<"Raw collection failed; any prior artifacts are retained: "<<error.what()<<'\n';
    return 1;
  }
}
