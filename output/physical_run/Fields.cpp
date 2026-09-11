#include "Types.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
namespace crash::output::physical_run {
Document StampDocument(const records::FrameStamp& s) {
    Document d;d.SetObject();
    Integer(d,"epoch",s.epoch);Integer(d,"base_epoch",s.base_epoch);Integer(d,"attempt",s.attempt);
    Number(d,"time",s.time);Number(d,"base_time",s.base_time);
    Number(d,"velocity_time",s.velocity_time);Number(d,"kick_dt",s.kick_dt);
    return d;
}
records::FrameStamp ReadStamp(const Value& v) {
    using namespace array_json;
    Keys(v,{"epoch","base_epoch","attempt","time","base_time","velocity_time","kick_dt"});
    return {UInt(v["epoch"]),UInt(v["base_epoch"]),UInt(v["attempt"]),Real(v["time"]),Real(v["base_time"]),
        Real(v["velocity_time"]),Real(v["kick_dt"])};
}
Document FileDocument(const records::RecordFile& f) {
    arrays::CheckRelativeName(f.file);arrays::CheckHash(f.sha256);
    Require(f.bytes<=kArtifactFileCap && (f.bytes || f.sha256==Sha256({})),"Invalid physical record size/hash");
    Document d;d.SetObject();String(d,"file",f.file);String(d,"sha256",f.sha256);Integer(d,"bytes",f.bytes);return d;
}
records::RecordFile ReadFileRecord(const Value& v) {
    using namespace array_json;
    Keys(v,{"file","sha256","bytes"});
    records::RecordFile f{Text(v["file"]),Text(v["sha256"]),static_cast<std::size_t>(UInt(v["bytes"]))};
    FileDocument(f);return f;
}
std::string ReadFile(const std::filesystem::path& root,const records::RecordFile& f,std::size_t cap) {
    FileDocument(f);Require(f.bytes<=cap,"Physical record exceeds read cap");
    const auto path=arrays::CheckedPath(root,f.file,true);
    Require(std::filesystem::file_size(path)==f.bytes,"Physical record byte count differs");
    auto bytes=ReadBounded(path,f.bytes);
    Require(bytes.size()==f.bytes && Sha256(bytes)==f.sha256,"Physical record identity differs");return bytes;
}
records::RecordFile WriteDocument(const std::filesystem::path& root,const std::string& name,const Document& d,std::size_t cap) {
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(d.Accept(writer) && buffer.GetSize()<=cap,"Physical metadata exceeds cap");
    const std::string bytes(buffer.GetString(),buffer.GetSize());
    const records::RecordFile f{name,Sha256(bytes),bytes.size()};
    FileDocument(f);WriteBytes(arrays::CheckedPath(root,name,false),bytes);return f;
}
} // namespace crash::output::physical_run
