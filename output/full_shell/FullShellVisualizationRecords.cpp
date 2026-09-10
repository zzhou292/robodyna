#include "FullShellVisualizationRecords.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <algorithm>
#include <cmath>

namespace crash::output::full_shell {
void CheckFrame(const Context& c,const FrameInput& f) {
    CheckStamp(c,f.stamp);
    Require(f.position_values==3*c.nodes()&&f.plastic_values==c.points()&&f.position_xyz&&
        (f.plastic_points||!f.plastic_values),"Incomplete frame value extent");
    for(std::size_t i=0;i<f.position_values;++i)Require(std::isfinite(f.position_xyz[i]),"Nonfinite accepted position");
    for(std::size_t i=0;i<f.plastic_values;++i)
        Require(std::isfinite(f.plastic_points[i])&&f.plastic_points[i]>=0,"Invalid native plastic strain");
}
RecordFile WriteFrame(const std::filesystem::path& root,const std::string& stem,const Context& c,const FrameInput& f) {
    arrays::CheckRelativeName(stem);Require(stem.find('/')==std::string::npos&&stem.size()<=128,"Invalid frame stem");
    CheckFrame(c,f);
    const arrays::Layout xl{arrays::Scalar::Float64,c.nodes(),3,{"x_m","y_m","z_m"}};
    const arrays::Layout pl{arrays::Scalar::Float64,c.points(),1,{"native_equivalent_plastic_strain"}};
    const auto xb=arrays::Encode(xl,f.position_xyz,f.position_values,c.limits().arrays);
    const auto pb=arrays::Encode(pl,f.plastic_points,f.plastic_values,c.limits().arrays);
    FrameDescription d{c.identity(),f.stamp,c.fixed_dt(),c.point_layout_sha256(),c.nodes(),c.parents().size(),c.points(),
        {stem+".positions.bin",xl,xb.size(),Sha256(xb)},{stem+".plastic.bin",pl,pb.size(),Sha256(pb)}};
    auto doc=FrameDocument(c,d);rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    Require(doc.Accept(writer),"Could not encode frame metadata");std::string json(buffer.GetString(),buffer.GetSize());
    Require(json.size()<=FrameMetadataByteCap,"Frame metadata exceeds capacity");
    RecordFile result{stem+".frame.json",Sha256(json),json.size()};
    // Any existing late destination rejects before writing either large array.
    const auto xfile=arrays::CheckedPath(root,d.positions.file,false),pfile=arrays::CheckedPath(root,d.plastic.file,false);
    const auto metadata=arrays::CheckedPath(root,result.file,false);
    output::WriteBytes(xfile,xb);output::WriteBytes(pfile,pb);output::WriteBytes(metadata,json);
    return result;
}
FrameRecord ReadFrame(const std::filesystem::path& root,const Context& c,const RecordFile& file,const FrameStamp& expected) {
    CheckStamp(c,expected);
    arrays::CheckHash(file.sha256);Require(file.bytes&&file.bytes<=FrameMetadataByteCap,"Invalid frame metadata size");
    const auto path=arrays::CheckedPath(root,file.file,true);
    Require(std::filesystem::file_size(path)==file.bytes,"Frame metadata file size mismatch");
    const auto bytes=output::ReadBounded(path,file.bytes);
    Require(bytes.size()==file.bytes&&Sha256(bytes)==file.sha256,"Frame metadata hash/size mismatch");
    auto doc=array_json::Parse(bytes,FrameMetadataByteCap);const auto description=ParseFrameDocument(c,doc);
    const auto& s=description.stamp;
    Require(s.epoch==expected.epoch&&s.base_epoch==expected.base_epoch&&s.attempt==expected.attempt&&
        Bits(s.time)==Bits(expected.time)&&Bits(s.base_time)==Bits(expected.base_time)&&
        Bits(s.velocity_time)==Bits(expected.velocity_time)&&Bits(s.kick_dt)==Bits(expected.kick_dt),
        "Frame does not match the expected accepted index phase");
    Require(file.file!=description.positions.file&&file.file!=description.plastic.file,"Frame metadata aliases a value array");
    FrameRecord frame{description.stamp,arrays::Read<double>(root,description.positions,c.limits().arrays),
        arrays::Read<double>(root,description.plastic,c.limits().arrays)};
    CheckFrame(c,{frame.stamp,frame.position_xyz.data(),frame.position_xyz.size(),frame.plastic_points.data(),frame.plastic_points.size()});
    return frame;
}
std::vector<std::optional<double>> ParentPlasticMaxima(const Context& c,const FrameRecord& f) {
    CheckFrame(c,{f.stamp,f.position_xyz.data(),f.position_xyz.size(),f.plastic_points.data(),f.plastic_points.size()});
    std::vector<std::optional<double>> result;result.reserve(c.parents().size());
    for(std::size_t i=0;i<c.parents().size();++i) {
        const auto begin=c.point_offsets()[i],end=c.point_offsets()[i+1];
        if(begin==end)result.emplace_back();
        else result.emplace_back(*std::max_element(f.plastic_points.begin()+begin,f.plastic_points.begin()+end));
    }
    return result;
}
} // namespace crash::output::full_shell
