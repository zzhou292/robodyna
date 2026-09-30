#include "Internal.h"

namespace crash::output::full_shell::activity {
struct ActivityRecord::Data {
    explicit Data(const Context& c):context(c) {}
    Context context;
    FrameStamp stamp;
    std::vector<std::uint64_t> words;
    std::size_t active=0;
};
namespace detail {
std::size_t WordCount(std::size_t n) {Require(n&&n<=1048576,"Invalid activity parent count");return (n+63)/64;}
arrays::Layout Layout(std::size_t n) {return {arrays::Scalar::UInt64,WordCount(n),1,{"parent_activity_bits"}};}
std::size_t Preflight(const Context& c,Limits limits) {
    Require(limits.host_bytes&&limits.host_bytes<=512*1024*1024,"Invalid activity host byte cap");
    const auto bytes=arrays::ByteCount(Layout(c.parents().size()),c.limits().arrays);
    std::size_t budget=0;
    const auto add=[&](std::size_t n,std::size_t width) {
        Require(n<=(limits.host_bytes-budget)/width,"Activity startup exceeds host byte cap");budget+=n*width;
    };
    add(c.retained_payload_bytes(),1);add(sizeof(ActivityRecord::Data)+sizeof(ActivityRecord),1);
    add(c.parents().size(),sizeof(std::uint8_t)); // Declared input range, not an owned solver array.
    add(bytes,4);add(MetadataByteCap,16); // Packed/read/codec buffers and bounded JSON scratch.
    return budget;
}
void CheckWords(const Context& c,const std::vector<std::uint64_t>& words) {
    Require(words.size()==WordCount(c.parents().size()),"Activity word count differs from source parents");
    const auto tail=c.parents().size()%64;
    Require(!tail||(words.back()>>tail)==0,"Activity unused high padding bits are nonzero");
}
}
ActivityRecord ActivityRecord::FromWords(const Context& c,FrameStamp stamp,std::vector<std::uint64_t> words) {
    CheckStamp(c,stamp);detail::CheckWords(c,words);
    auto next=std::make_shared<Data>(c);next->stamp=stamp;next->words=std::move(words);
    for(std::size_t i=0;i<c.parents().size();++i)next->active+=(next->words[i/64]>>(i%64))&1;
    return ActivityRecord(std::move(next));
}
ActivityRecord ActivityRecord::Create(const Context& c,const ActivityInput& input,const FrameStamp& expected,Limits limits) {
    detail::Preflight(c,limits);CheckStamp(c,expected);CheckStamp(c,input.stamp);
    Require(SameIdentity(c.identity(),input.identity)&&input.point_layout_sha256==c.point_layout_sha256(),
            "Activity source/owner/layout identity mismatch");
    Require(SameStamp(input.stamp,expected),"Activity does not match expected accepted phase");
    Require(input.parents==c.parents().size()&&input.parent_active,"Incomplete activity parent extent");
    std::vector<std::uint64_t> words(detail::WordCount(input.parents),0);
    for(std::size_t i=0;i<input.parents;++i) {
        const auto flag=input.parent_active[i];Require(flag<=1,"Accepted activity must be exactly zero or one");
        words[i/64]|=std::uint64_t(flag)<<(i%64);
    }
    return FromWords(c,input.stamp,std::move(words));
}
const Context& ActivityRecord::context() const noexcept {return data_->context;}
const FrameStamp& ActivityRecord::stamp() const noexcept {return data_->stamp;}
const std::vector<std::uint64_t>& ActivityRecord::words() const noexcept {return data_->words;}
std::size_t ActivityRecord::active_count() const noexcept {return data_->active;}
bool ActivityRecord::active(std::size_t i) const {
    Require(i<context().parents().size(),"Activity parent index is out of range");return (words()[i/64]>>(i%64))&1;
}
} // namespace crash::output::full_shell::activity
