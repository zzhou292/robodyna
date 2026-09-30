#include "BoundedArrayIO.h"
#include "BoundedArrayJson.h"

namespace crash::output::arrays {
Document DescriptorDocument(const Descriptor& d,Limits cap) {
    CheckDescriptor(d,cap);Document doc;doc.SetObject();
    String(doc,"file",d.file);String(doc,"dtype",Dtype(d.layout.scalar));
    Value shape(rapidjson::kArrayType);shape.PushBack(Value().SetUint64(d.layout.rows),doc.GetAllocator());
    shape.PushBack(Value().SetUint64(d.layout.columns),doc.GetAllocator());doc.AddMember("shape",shape,doc.GetAllocator());
    Integer(doc,"bytes",d.bytes);String(doc,"sha256",d.sha256);Value fields;
    if(!d.layout.fields.empty()) {
        fields.SetArray();
        for(const auto& f:d.layout.fields)fields.PushBack(Value(f.c_str(),doc.GetAllocator()),doc.GetAllocator());
    }
    doc.AddMember("fields",fields,doc.GetAllocator());return doc;
}
Descriptor ParseDescriptor(const Value& v,Limits cap) {
    using namespace array_json;Keys(v,{"file","dtype","shape","bytes","sha256","fields"});
    Descriptor d;d.file=Text(v["file"]);d.sha256=Text(v["sha256"]);const auto size=UInt(v["bytes"]);
    Require(size<=cap.file_bytes,"Array declared size exceeds cap");d.bytes=static_cast<std::size_t>(size);
    const auto dtype=Text(v["dtype"]);bool found=false;
    for(auto type:{Scalar::UInt16,Scalar::UInt32,Scalar::UInt64,Scalar::Int32,Scalar::Float64})
        if(dtype==Dtype(type)){d.layout.scalar=type;found=true;break;}
    Require(found,"Unsupported array dtype");
    const auto& shape=v["shape"];Require(shape.IsArray()&&shape.Size()==2,"Invalid array shape");
    d.layout.rows=UInt(shape[0]);const auto cols=UInt(shape[1]);
    Require(cols<=64,"Array column capacity exceeded");d.layout.columns=static_cast<std::size_t>(cols);
    const auto& fields=v["fields"];
    Require(fields.IsNull()||(fields.IsArray()&&fields.Size()==cols),"Invalid array field declarations");
    if(fields.IsArray())for(const auto& f:fields.GetArray())d.layout.fields.push_back(Text(f));
    CheckDescriptor(d,cap);return d;
}
} // namespace crash::output::arrays
