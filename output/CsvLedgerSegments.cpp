#include "CsvLedgerSegments.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

namespace crash::output {
namespace {
bool SafeFile(const std::string& name) {
    if(name.size()<5||name.size()>120||name.substr(name.size()-4)!=".csv")return false;
    return std::all_of(name.begin(),name.end()-4,[](unsigned char c) {
        return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-';
    });
}
std::string SegmentName(const std::string& logical,std::size_t index) {
    if(!index)return logical;
    std::ostringstream name;
    name<<logical.substr(0,logical.size()-4)<<'-'<<std::setw(4)<<std::setfill('0')<<index<<".csv";
    return name.str();
}
std::size_t HeaderColumns(const std::string& header) {
    Require(!header.empty()&&header.size()<=kCsvLedgerHeaderCap&&header.back()=='\n',"Invalid bounded CSV header");
    std::size_t columns=1,length=0,start=0;
    std::set<std::string> names;
    for(std::size_t i=0;i+1<header.size();++i) {
        const unsigned char c=header[i];
        if(c==',') {
            Require(length&&names.insert(header.substr(start,length)).second,"Empty or duplicate CSV header field");
            start=i+1;length=0;++columns;
        }
        else {
            Require((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_',"Invalid CSV header field");
            ++length;
        }
    }
    Require(length&&names.insert(header.substr(start,length)).second&&columns<=kCsvLedgerColumnCap,"Invalid CSV header column count");
    return columns;
}
void Basic(const CsvLedgerPlan& p) {
    Require(SafeFile(p.logical_file)&&p.header_sha256.size()==64&&
        p.header_sha256.find_first_not_of("0123456789abcdef")==std::string::npos&&
        p.column_count&&p.column_count<=kCsvLedgerColumnCap&&p.max_row_bytes>=2*p.column_count&&
        p.max_row_bytes<=kCsvLedgerRowCap&&p.interval_count&&
        !p.segments.empty()&&p.segments.size()<=kCsvLedgerSegmentCap,"Invalid CSV ledger plan");
    std::uint64_t next=1;
    std::size_t total=0,header_size=0;
    for(std::size_t i=0;i<p.segments.size();++i) {
        const auto& s=p.segments[i];
        Require(SafeFile(s.file)&&s.file==SegmentName(p.logical_file,i)&&s.first_epoch==next&&
            s.row_count&&s.last_epoch>=s.first_epoch&&s.last_epoch<=p.interval_count&&
            s.row_count==s.last_epoch-s.first_epoch+1&&s.byte_cap<=kArtifactFileCap&&
            s.row_count<=s.byte_cap/p.max_row_bytes,"Invalid CSV segment name/range/capacity");
        const auto header=s.byte_cap-static_cast<std::size_t>(s.row_count)*p.max_row_bytes;
        Require(header&&header<=kCsvLedgerHeaderCap&&(!i||header==header_size),"CSV segment header capacities disagree");
        header_size=header;
        Require(s.byte_cap<=kArtifactTotalCap-total,"CSV ledger exceeds aggregate byte cap");
        total+=s.byte_cap;
        Require(s.last_epoch<std::numeric_limits<std::uint64_t>::max(),"CSV epoch range overflow");
        next=s.last_epoch+1;
    }
    Require(next-1==p.interval_count&&p.total_bytes==total,"CSV ledger has missing rows or incorrect total capacity");
}
void Collection(const CsvLedgerPlan* plans,std::size_t count) {
    Require(plans&&count&&count<=kCsvLedgerCountCap,"Invalid CSV ledger collection");
    std::set<std::string> names,logical;
    std::size_t total=0;
    for(std::size_t i=0;i<count;++i) {
        Basic(plans[i]);
        Require(logical.insert(plans[i].logical_file).second,"Duplicate logical CSV ledger");
        for(const auto& segment:plans[i].segments)Require(names.insert(segment.file).second,"Overlapping CSV ledger filenames");
        Require(plans[i].total_bytes<=kArtifactTotalCap-total,"CSV ledger collection exceeds aggregate byte cap");
        total+=plans[i].total_bytes;
    }
}
const Value& Member(const Value& object,const char* key) {
    Require(object.IsObject(),"CSV metadata must be an object");
    const Value* found=nullptr;
    for(auto i=object.MemberBegin();i!=object.MemberEnd();++i)
        if(i->name.GetStringLength()==std::char_traits<char>::length(key)&&i->name==key) {
            Require(!found,"Duplicate required CSV metadata member");found=&i->value;
        }
    Require(found,"Missing required CSV metadata member");
    return *found;
}
std::uint64_t IntegerValue(const Value& object,const char* key) {
    const auto& value=Member(object,key);
    Require(value.IsUint64(),"CSV metadata integer has wrong type");return value.GetUint64();
}
std::size_t SizeValue(const Value& object,const char* key) {
    const auto value=IntegerValue(object,key);
    Require(value<=std::numeric_limits<std::size_t>::max(),"CSV metadata integer exceeds host size");
    return static_cast<std::size_t>(value);
}
std::string TextValue(const Value& object,const char* key) {
    const auto& value=Member(object,key);
    Require(value.IsString()&&value.GetStringLength()<=128,"CSV metadata text has wrong type or length");
    return {value.GetString(),value.GetStringLength()};
}
void AddNumber(Value& value,const char* key,std::uint64_t number,Document::AllocatorType& allocator) {
    value.AddMember(Value(key,allocator).Move(),Value().SetUint64(number),allocator);
}
void AddText(Value& value,const char* key,const std::string& text,Document::AllocatorType& allocator) {
    value.AddMember(Value(key,allocator).Move(),Value(text.c_str(),static_cast<rapidjson::SizeType>(text.size()),allocator).Move(),allocator);
}
} // namespace

CsvLedgerPlan PlanCsvLedger(const std::string& logical,const std::string& header,
                           std::uint64_t intervals,std::size_t row_bytes,std::size_t file_cap) {
    Require(SafeFile(logical)&&intervals&&file_cap<=kArtifactFileCap&&file_cap>header.size(),"Invalid CSV ledger dimensions or file cap");
    const auto columns=HeaderColumns(header);
    Require(row_bytes>=2*columns&&row_bytes<=kCsvLedgerRowCap,"Invalid maximum encoded CSV row size");
    const std::size_t capacity=(file_cap-header.size())/row_bytes;
    Require(capacity&&intervals<=static_cast<std::uint64_t>(capacity)*kCsvLedgerSegmentCap,"CSV ledger exceeds bounded segment capacity");
    CsvLedgerPlan plan;
    plan.logical_file=logical;plan.header_sha256=Sha256(header);plan.column_count=columns;
    plan.max_row_bytes=row_bytes;plan.interval_count=intervals;
    for(std::uint64_t first=1;first<=intervals;) {
        CsvLedgerSegment segment;
        segment.file=SegmentName(logical,plan.segments.size());segment.first_epoch=first;
        segment.row_count=std::min<std::uint64_t>(capacity,intervals-first+1);
        segment.last_epoch=first+segment.row_count-1;
        segment.byte_cap=header.size()+static_cast<std::size_t>(segment.row_count)*row_bytes;
        plan.total_bytes+=segment.byte_cap;plan.segments.push_back(std::move(segment));
        first=plan.segments.back().last_epoch+1;
    }
    Basic(plan);return plan;
}
bool SameCsvLedgerPlan(const CsvLedgerPlan& a,const CsvLedgerPlan& b) noexcept {
    if(a.logical_file!=b.logical_file||a.header_sha256!=b.header_sha256||a.column_count!=b.column_count||
       a.max_row_bytes!=b.max_row_bytes||a.interval_count!=b.interval_count||a.total_bytes!=b.total_bytes||
       a.segments.size()!=b.segments.size())return false;
    for(std::size_t i=0;i<a.segments.size();++i) {
        const auto& x=a.segments[i];const auto& y=b.segments[i];
        if(x.file!=y.file||x.first_epoch!=y.first_epoch||x.last_epoch!=y.last_epoch||x.row_count!=y.row_count||x.byte_cap!=y.byte_cap)return false;
    }
    return true;
}
void ValidateCsvLedgerPlan(const CsvLedgerPlan& plan,const std::string& header,std::size_t cap) {
    Basic(plan);
    const auto expected=PlanCsvLedger(plan.logical_file,header,plan.interval_count,plan.max_row_bytes,cap);
    Require(SameCsvLedgerPlan(plan,expected),"CSV metadata differs from its authenticated header and deterministic layout");
}
std::vector<CsvLedgerPlan> ParseCsvLedgerSegments(const Value& value) {
    Require(value.IsArray()&&value.Size()&&value.Size()<=kCsvLedgerCountCap,"Invalid CSV ledger metadata array");
    std::vector<CsvLedgerPlan> plans;
    for(const auto& item:value.GetArray()) {
        CsvLedgerPlan p;
        p.logical_file=TextValue(item,"logical_file");p.header_sha256=TextValue(item,"header_sha256");
        p.column_count=SizeValue(item,"column_count");p.max_row_bytes=SizeValue(item,"max_row_bytes");
        p.interval_count=IntegerValue(item,"interval_count");
        const auto& segments=Member(item,"segments");
        Require(segments.IsArray()&&segments.Size()&&segments.Size()<=kCsvLedgerSegmentCap,"Invalid CSV segment metadata array");
        for(const auto& entry:segments.GetArray()) {
            CsvLedgerSegment s;
            s.file=TextValue(entry,"file");s.first_epoch=IntegerValue(entry,"first_epoch");s.last_epoch=IntegerValue(entry,"last_epoch");
            s.row_count=IntegerValue(entry,"row_count");s.byte_cap=SizeValue(entry,"byte_cap");
            Require(s.byte_cap<=kArtifactTotalCap-p.total_bytes,"CSV metadata total capacity overflow");
            p.total_bytes+=s.byte_cap;p.segments.push_back(std::move(s));
        }
        plans.push_back(std::move(p));
    }
    Collection(plans.data(),plans.size());return plans;
}
void AppendCsvLedgerSegments(Document& document,const CsvLedgerPlan* plans,std::size_t count) {
    Require(document.IsObject()&&!document.HasMember(kCsvLedgerSegmentsField),"CSV ledger metadata is already present");
    Collection(plans,count);
    Value array(rapidjson::kArrayType);auto& allocator=document.GetAllocator();
    for(std::size_t i=0;i<count;++i) {
        const auto& p=plans[i];Value item(rapidjson::kObjectType),segments(rapidjson::kArrayType);
        AddText(item,"logical_file",p.logical_file,allocator);AddText(item,"header_sha256",p.header_sha256,allocator);
        AddNumber(item,"column_count",p.column_count,allocator);AddNumber(item,"max_row_bytes",p.max_row_bytes,allocator);
        AddNumber(item,"interval_count",p.interval_count,allocator);
        for(const auto& s:p.segments) {
            Value entry(rapidjson::kObjectType);
            AddText(entry,"file",s.file,allocator);AddNumber(entry,"first_epoch",s.first_epoch,allocator);
            AddNumber(entry,"last_epoch",s.last_epoch,allocator);AddNumber(entry,"row_count",s.row_count,allocator);
            AddNumber(entry,"byte_cap",s.byte_cap,allocator);segments.PushBack(entry,allocator);
        }
        item.AddMember("segments",segments,allocator);array.PushBack(item,allocator);
    }
    document.AddMember(Value(kCsvLedgerSegmentsField,allocator).Move(),array,allocator);
}
} // namespace crash::output
