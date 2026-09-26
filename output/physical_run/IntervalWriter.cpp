#include "IntervalIO.h"
#include <algorithm>
namespace crash::output::physical_run {
struct IntervalWriter::Data {
    Data(std::filesystem::path r,const records::Context& c):root(std::move(r)),context(c) {}
    std::filesystem::path root;
    records::Context context;
    Profile profile;
    interval::ChunkPlan plan;
    Sequence sequence;
    std::vector<std::uint64_t> integers;
    std::vector<double> reals;
    std::vector<Segment> segments;
    std::size_t buffered=0,columns=0,integer_columns=0;
    bool failed=false,finished=false;
};
IntervalWriter::IntervalWriter(std::filesystem::path root,const records::Context& c,Profile p,
    std::uint64_t planned,std::size_t file_cap,std::size_t host_cap) {
    const auto plan=interval::PlanChunks(planned,file_cap,kArtifactMaximumTotalCap,ExtraIntervalBytes(p));
    const auto rows=std::min(plan.rows_per_chunk,planned),columns=RealFields(p).size();
    const auto integer_columns=IntegerFields(p).size();
    Require(host_cap && host_cap<=256u<<20 && rows<=host_cap/(8*(integer_columns+columns)*3),
        "Physical interval staging/encoding exceeds host cap");
    for(std::size_t i=0;i<plan.chunks;++i) {
        arrays::CheckedPath(root,"interval-"+std::to_string(i)+".integers.bin",false);
        arrays::CheckedPath(root,"interval-"+std::to_string(i)+".reals.bin",false);
    }
    auto s=std::make_unique<Data>(std::move(root),c);
    s->profile=p;s->plan=plan;s->columns=columns;s->integer_columns=integer_columns;
    s->integers.resize(rows*integer_columns);s->reals.resize(rows*columns);s->segments.reserve(plan.chunks);data_=std::move(s);
}
IntervalWriter::~IntervalWriter()=default;
void IntervalWriter::Append(const Values& v) {
    auto& s=*data_;
    Require(!s.failed && !s.finished,"Physical interval writer is closed");
    const auto next=Advance(s.context,s.profile,s.plan.intervals,s.sequence,v);
    const auto& f=v.stamp;
    const std::uint64_t ints[]{v.owner,f.base_epoch,f.attempt,f.epoch};
    const double reals[]{f.base_time,f.time,f.velocity_time,f.kick_dt};
    std::copy_n(ints,4,s.integers.data()+s.integer_columns*s.buffered);
    std::copy_n(reals,4,s.reals.data()+s.columns*s.buffered);
    if(v.structural_limit_s)s.reals[s.columns*s.buffered+4]=*v.structural_limit_s;
    if(v.self_contact)EncodeSelfContact(*v.self_contact,
        s.integers.data()+s.integer_columns*s.buffered+4,
        s.reals.data()+s.columns*s.buffered+4+std::size_t(s.profile.structural_limit));
    if(v.native_contact)EncodeNativeContact(*v.native_contact,
        s.integers.data()+s.integer_columns*s.buffered+4,
        s.reals.data()+s.columns*s.buffered+4+std::size_t(s.profile.structural_limit));
    if(v.native_group)EncodeNativeGroup(*v.native_group,
        s.integers.data()+s.integer_columns*s.buffered+4,
        s.reals.data()+s.columns*s.buffered+4+std::size_t(s.profile.structural_limit));
    s.sequence=next;++s.buffered;
    if(s.buffered==s.plan.rows_per_chunk)try {Flush();} catch(...) {s.failed=true;throw;}
}
void IntervalWriter::Flush() {
    auto& s=*data_;if(!s.buffered)return;
    const auto stem="interval-"+std::to_string(s.segments.size());
    arrays::CheckedPath(s.root,stem+".integers.bin",false);arrays::CheckedPath(s.root,stem+".reals.bin",false);
    const arrays::Limits limits{s.plan.file_byte_cap,UINT32_MAX,64};
    Segment segment;
    segment.first_epoch=s.sequence.last.epoch-s.buffered+1;segment.rows=s.buffered;
    segment.integers=arrays::Write(s.root,stem+".integers.bin",{arrays::Scalar::UInt64,s.buffered,s.integer_columns,IntegerFields(s.profile)},
        s.integers.data(),s.buffered*s.integer_columns,limits);
    segment.reals=arrays::Write(s.root,stem+".reals.bin",{arrays::Scalar::Float64,s.buffered,s.columns,RealFields(s.profile)},
        s.reals.data(),s.buffered*s.columns,limits);
    s.segments.push_back(std::move(segment));s.buffered=0;
}
std::vector<Segment> IntervalWriter::Finish() {
    auto& s=*data_;Require(!s.failed && !s.finished,"Physical interval writer is closed");
    try {Flush();} catch(...) {s.failed=true;throw;}
    s.finished=true;return std::move(s.segments);
}
const Sequence& IntervalWriter::sequence() const noexcept {return data_->sequence;}
std::size_t IntervalWriter::buffer_bytes() const noexcept {return 8*(data_->integers.size()+data_->reals.size());}
bool IntervalWriter::failed() const noexcept {return data_->failed;}
} // namespace crash::output::physical_run
