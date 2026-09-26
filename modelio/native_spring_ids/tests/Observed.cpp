#include "Observed.h"
#include "output/ArtifactIO.h"
#include <array>
#include <charconv>
#include <limits>
#include <map>
#include <set>
#include <string_view>
namespace crash::modelio::native_spring_ids::test {
namespace {
using output::Require;
std::string Read(const std::filesystem::path& path,std::size_t bytes,const char* sha) {
    auto data=output::ReadBounded(path,bytes);
    Require(data.size()==bytes,"Observed SPRING evidence byte extent changed");
    Require(output::Sha256(data)==sha,"Observed SPRING evidence SHA256 changed");
    return data;
}
std::uint64_t Integer(std::string_view field) {
    std::uint64_t value=0;
    const auto parsed=std::from_chars(field.data(),field.data()+field.size(),value);
    Require(!field.empty() && parsed.ec==std::errc{} && parsed.ptr==field.data()+field.size() && value>0,
        "Observed SPRING evidence integer is not canonical positive uint64");
    return value;
}
std::size_t Index(std::string_view field) {
    const auto value=Integer(field);
    Require(value<=std::numeric_limits<std::size_t>::max(),"Observed SPRING evidence index overflow");
    return static_cast<std::size_t>(value);
}
std::optional<std::size_t> OptionalIndex(std::string_view field) {
    return field.empty() ? std::nullopt : std::optional<std::size_t>{Index(field)};
}
template<std::size_t Columns,class Accept>
void Rows(const std::string& data,std::string_view header,Accept accept) {
    // Both pinned files use the unquoted CSV subset and CRLF. Keep this bounded
    // evidence decoder separate from production source keyword parsing.
    std::size_t cursor=0,line=0;
    while(cursor<data.size()) {
        const auto end=data.find('\n',cursor);
        Require(end!=std::string::npos && end>cursor && data[end-1]=='\r',"Observed CSV requires complete CRLF rows");
        const std::string_view text(data.data()+cursor,end-cursor-1);cursor=end+1;
        if(line++==0){Require(text==header,"Observed CSV schema changed");continue;}
        std::array<std::string_view,Columns> fields;
        std::size_t begin=0;
        for(std::size_t i=0;i<Columns;++i) {
            const auto comma=text.find(',',begin);
            Require((i+1==Columns)==(comma==std::string_view::npos),"Observed CSV column count changed");
            const auto stop=comma==std::string_view::npos?text.size():comma;
            fields[i]=text.substr(begin,stop-begin);
            Require(fields[i].find('"')==std::string_view::npos,"Observed CSV quoted field is outside its pinned format");
            begin=stop+1;
        }
        accept(fields);
    }
}
}
Observed LoadObserved(const std::filesystem::path& directory) {
    const auto generated=Read(directory/"generated-springs.csv",217584,
        "3a7f3457dc4457d661165a1efbad2159e84e007e9323e3708ef50d5c167b5a21");
    const auto native=Read(directory/"native-spring-table.csv",1229110,
        "01a5befe63add3d7c51e6d7bf9c937c291071b4ea72f9e34c41b53ebc1982c82");
    Observed result;result.generated.reserve(2872);result.native_table.reserve(7349);
    std::set<std::pair<std::string,std::uint64_t>> source_keys;
    std::set<std::uint64_t> generated_ids;
    std::size_t welds=0,joints=0;
    Rows<8>(generated,"keyword,source_id,native_id,node1,node2,output_line,source_keyword_line,source_id_line",
        [&](const auto& fields) {
            Require(result.generated.size()<2872,"Observed generated SPRING row bound exceeded");
            ObservedGenerated row;
            row.keyword=fields[0];row.source_id=Integer(fields[1]);row.native_id=Integer(fields[2]);
            row.node1=Integer(fields[3]);row.node2=Integer(fields[4]);row.output_line=Index(fields[5]);
            row.source_keyword_line=OptionalIndex(fields[6]);row.source_id_line=OptionalIndex(fields[7]);
            if(row.keyword=="*CONSTRAINED_SPOTWELD_ID")++welds;
            else {
                Require(row.keyword=="*CONSTRAINED_JOINT_CYLINDRICAL_ID" ||
                    row.keyword=="*CONSTRAINED_JOINT_REVOLUTE_ID" || row.keyword=="*CONSTRAINED_JOINT_SPHERICAL_ID",
                    "Observed generated SPRING keyword changed");++joints;
            }
            Require(source_keys.emplace(row.keyword,row.source_id).second && generated_ids.insert(row.native_id).second,
                "Observed generated SPRING identity is duplicated");
            result.generated.push_back(std::move(row));
        });
    std::map<std::uint64_t,std::size_t> native_ids;
    Rows<7>(native,"local,native_id,property_id,node1,node2,output_line,raw_line",[&](const auto& fields) {
        Require(result.native_table.size()<7349,"Observed native SPRING row bound exceeded");
        ObservedNative row;
        row.local=Index(fields[0]);row.native_id=Integer(fields[1]);row.property_id=Integer(fields[2]);
        row.node1=Integer(fields[3]);row.node2=Integer(fields[4]);row.output_line=Index(fields[5]);row.raw_line=fields[6];
        Require(row.local==result.native_table.size()+1 && !row.raw_line.empty(),"Observed native SPRING local row order changed");
        Require(native_ids.emplace(row.native_id,result.native_table.size()).second,"Observed native SPRING identity is duplicated");
        result.native_table.push_back(std::move(row));
    });
    Require(result.generated.size()==2872 && result.native_table.size()==7349 && welds==2828 && joints==44,
        "Observed SPRING evidence is incomplete");
    for(const auto& row:result.generated) {
        const auto match=native_ids.find(row.native_id);
        Require(match!=native_ids.end(),"Observed generated SPRING missing from native table");
        const auto& full=result.native_table[match->second];
        Require(full.node1==row.node1 && full.node2==row.node2,"Observed SPRING endpoint identity differs between evidence tables");
    }
    return result;
}
}
