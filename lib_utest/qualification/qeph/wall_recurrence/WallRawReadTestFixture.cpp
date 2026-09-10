#include "WallRawReadTestFixture.h"
#include <map>

namespace tl::qualification::qeph::wall_recurrence::read_test {
RawReadBinding Clone(const std::filesystem::path& source,const std::filesystem::path& destination,const Mutation& mutate) {
  io::Require(std::filesystem::create_directory(destination),"Clone destination must be new");
  const auto old_index=rt::Read(source/"index.json"); std::vector<std::string> names;
  for(const auto& f:old_index["files"].GetArray()) names.emplace_back(f["name"].GetString());
  names.push_back("index.json");
  std::vector<RawFileReceipt> files; std::map<std::string,std::string> hashes; std::size_t total=0;
  for(const auto& name:names) {
    std::string bytes=io::ReadBounded(source/name,RawFileByteCap);
    if(name!="provenance.json") {
      auto d=rt::Read(source/name);
      if(d.HasMember("model_sha256")) d["model_sha256"].SetString(hashes.at("model.json").c_str(),d.GetAllocator());
      if(d.HasMember("native_matrix_sha256"))
        d["native_matrix_sha256"].SetString(hashes.at(d["native_matrix_file"].GetString()).c_str(),d.GetAllocator());
      if(d.HasMember("files")) {
        io::Value entries(rapidjson::kArrayType);
        for(const auto& f:files) {
          io::Document entry; entry.SetObject(); io::String(entry,"name",f.name); io::String(entry,"sha256",f.sha256); io::Integer(entry,"bytes",f.bytes);
          raw_detail::Append(d,entries,entry);
        }
        d["files"]=std::move(entries); d["bytes_before_this_index"].SetUint64(total);
      }
      mutate(name,d); bytes=EncodeRawJson(d);
    }
    const auto hash=io::Sha256(bytes); io::WriteBytes(destination/name,bytes);
    files.push_back({name,hash,bytes.size()}); hashes[name]=hash; total+=bytes.size();
  }
  return Binding(destination);
}
} // namespace tl::qualification::qeph::wall_recurrence::read_test
