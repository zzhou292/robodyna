#include "Internal.h"
#include <climits>
namespace crash::modelio::solid_control_packets::detail {
namespace json=output::array_json;
namespace {
std::size_t Size(const output::Value& value,std::size_t cap) {
    const auto x=json::UInt(value);Require(x<=cap,"Native packet integer exceeds declared bounds");return static_cast<std::size_t>(x);
}
control::Family FamilyName(const output::Value& value) {
    const auto name=json::Text(value);
    if(name=="Solid18")return control::Family::Solid18;
    if(name=="Solid24")return control::Family::Solid24;
    if(name=="Solid6z")return control::Family::Solid6z;
    if(name=="Solid18Law44")return control::Family::Solid18Law44;
    if(name=="Solid18Law90")return control::Family::Solid18Law90;
    throw std::runtime_error("Unsupported native packet family");
}
std::uint64_t Id(const output::Value& value) {
    const auto id=json::UInt(value);Require(id&&id<=INT_MAX,"Invalid native external source identifier");return id;
}
void Array(const output::Value& value,std::size_t cap) {
    Require(value.IsArray()&&value.Size()<=cap,"Native packet array exceeds bounded extent");
}
}
Values Read(const std::string& bytes,const Artifact& artifact,Limits limits) {
    (void)Preflight(bytes.size(),limits);
    auto d=json::Parse(bytes,limits.file_bytes);
    json::Keys(d,{"schema","status","case_profile","source_instance_id_binding","units","native_nvsiz",
        "compiled_mvsiz","partitions","packets","ordered_element_ids","parents","controlled_count",
        "solid_count","packet_count","raw_packets","actual_igrounc","provenance"});
    Require(json::Text(d["schema"])=="robo_dyna.native_solid_control_input.v1"&&
        json::Text(d["status"])=="offline_actual_native_packet_export"&&
        json::Text(d["case_profile"])==artifact.case_profile,"Native packet schema or case profile differs");
    Values out;const auto& units=d["units"];json::Keys(units,{"length_m","mass_kg","time_s"});
    out.units={json::Real(units["length_m"]),json::Real(units["mass_kg"]),json::Real(units["time_s"])};
    tl::fea::solid_common::distortion::units_detail::Factors factors;
    Require(tl::fea::solid_common::distortion::units_detail::Make(out.units,factors),"Unsupported native packet units");
    out.native_nvsiz=Size(d["native_nvsiz"],4096);out.compiled_mvsiz=Size(d["compiled_mvsiz"],4097);
    Array(d["parents"],limits.parents);Array(d["partitions"],limits.partitions);
    Array(d["packets"],limits.packets);Array(d["ordered_element_ids"],limits.parents);
    const auto count=d["parents"].Size();
    Require(count&&Size(d["solid_count"],limits.parents)==count&&d["ordered_element_ids"].Size()==count&&
        Size(d["packet_count"],limits.packets)==d["packets"].Size(),"Native packet complete census differs");
    out.parents.reserve(count);out.parent_families.reserve(count);
    for(const auto& p:d["parents"].GetArray()) {
        json::Keys(p,{"element_id","part_id","section_id","material_id","native_property_id","icontrol","family"});
        out.parents.push_back({Id(p["element_id"]),Id(p["part_id"]),Id(p["section_id"]),Id(p["material_id"]),
            Id(p["native_property_id"]),static_cast<std::uint32_t>(Size(p["icontrol"],1))});
        out.parent_families.push_back(FamilyName(p["family"]));out.controlled_count+=out.parents.back().icontrol;
    }
    Require(out.controlled_count==Size(d["controlled_count"],count),"Native controlled census differs");
    out.partitions.reserve(d["partitions"].Size());
    for(const auto& p:d["partitions"].GetArray()) {
        json::Keys(p,{"partition_id","packet_begin","packet_count","member_begin","member_count"});
        out.partitions.push_back({json::UInt(p["partition_id"]),Size(p["packet_begin"],limits.packets),
            Size(p["packet_count"],limits.packets),Size(p["member_begin"],count),Size(p["member_count"],count)});
    }
    out.packets.reserve(d["packets"].Size());
    for(const auto& p:d["packets"].GetArray()) {
        json::Keys(p,{"group_id","native_first","member_begin","member_count","family","material_id","native_property_id","icontrol"});
        out.packets.push_back({Id(p["group_id"]),Size(p["native_first"],count),Size(p["member_begin"],count),
            Size(p["member_count"],count),FamilyName(p["family"]),Id(p["material_id"]),Id(p["native_property_id"]),
            static_cast<std::uint32_t>(Size(p["icontrol"],1))});
    }
    out.ordered_element_ids.reserve(count);
    for(const auto& id:d["ordered_element_ids"].GetArray())out.ordered_element_ids.push_back(Id(id));
    Require(Owned(out)<=limits.retained_bytes,"Native packet owned payload exceeds cap");return out;
}
} // namespace crash::modelio::solid_control_packets::detail
