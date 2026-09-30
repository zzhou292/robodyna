#include "WallRawReport.h"
#include "WallRawJson.h"
#include "lib_utest/qualification/qeph/free_response/RecurrenceReport.h"
#include <cmath>
#include <set>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
namespace {
void CheckJson(const io::Value& value,unsigned depth,std::size_t& count) {
  io::Require(depth<=32&&++count<=8192,"Unbounded provenance JSON structure");
  if(value.IsObject()) {
    std::set<std::string> keys;
    for(auto it=value.MemberBegin();it!=value.MemberEnd();++it) {
      io::Require(keys.emplace(it->name.GetString(),it->name.GetStringLength()).second,"Duplicate provenance JSON key");
      CheckJson(it->value,depth+1,count);
    }
  } else if(value.IsArray()) for(const auto& item:value.GetArray()) CheckJson(item,depth+1,count);
  else if(value.IsNumber()) io::Require(std::isfinite(value.GetDouble()),"Nonfinite provenance number");
}
bool Hex(const std::string& text) {
  return text.size()==64&&text.find_first_not_of("0123456789abcdef")==std::string::npos;
}
unsigned Identity(unsigned cells,double velocity) {
  io::Require((cells==1||cells==2)&&FrozenVelocity(velocity)&&!(velocity==0&&std::signbit(velocity)),"Invalid raw job identity");
  return 3*(cells-1)+(velocity<0?0u:(velocity==0?1u:2u));
}
}
io::Document ParseRawProvenance(const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=ProvenanceByteCap,"Missing or oversized exact raw provenance");
  io::Document d;
  d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
  io::Require(!d.HasParseError()&&d.IsObject()&&!d.ObjectEmpty(),"Malformed raw provenance object");
  std::size_t count=0; CheckJson(d,0,count); return d;
}
std::string EncodeRawJson(const io::Document& d) {
  // Existing compact finite-JSON writer and 32 MiB cap; marker objects are
  // ordinary valid JSON, not forbidden NaN/Infinity numeric tokens.
  return recurrence::EncodeReport(d);
}
std::size_t RawSetBytes(const std::array<RawJobReceipt,6>& receipts,std::size_t derived) {
  io::Require(derived<=ScreenSetByteCap,"Derived report bytes exceed shared screen cap");
  std::size_t total=derived; std::set<unsigned> jobs;
  for(const auto& receipt:receipts) {
    io::Require(jobs.insert(Identity(receipt.cells,receipt.normal_velocity)).second,"Duplicate raw fixture/boost job");
    io::Require(!receipt.files.empty()&&receipt.files.size()<=80,"Missing or unbounded raw receipt inventory");
    std::size_t bytes=0; std::set<std::string> names; bool final=false;
    for(const auto& f:receipt.files) {
      io::Require(!f.name.empty()&&f.name.size()<=64&&f.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-.")==std::string::npos&&
        f.name!="."&&f.name!=".."&&names.insert(f.name).second&&Hex(f.sha256)&&f.bytes>0&&f.bytes<=RawFileByteCap,
        "Malformed raw artifact receipt");
      io::Require(f.bytes<=ScreenSetByteCap-bytes,"Raw job receipt byte overflow"); bytes+=f.bytes;
      if(f.name=="index.json") final=true;
    }
    io::Require(names.count("provenance.json")==1&&bytes==receipt.total_bytes&&final==receipt.final_index_present&&
      (!receipt.collection_complete||receipt.final_index_present),"Inconsistent raw receipt completion/bytes");
    io::Require(bytes<=ScreenSetByteCap-total,"Combined raw and derived reports exceed 96 MiB"); total+=bytes;
  }
  return total;
}
} // namespace tl::qualification::qeph::wall_recurrence
