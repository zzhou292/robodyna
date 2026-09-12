#pragma once
#include "output/full_shell/activity/ActivityRecord.h"
#include "output/full_shell/IntervalChunkPlan.h"
#include <optional>
namespace crash::output::physical_run {
namespace records=full_shell;
inline constexpr const char* Schema="robo_dyna.physical_accepted_run.v1";
inline constexpr const char* IndexSchema="robo_dyna.physical_accepted_index.v1";
inline constexpr const char* ProfileSchema="robo_dyna.physical_observation_profile.v1";
inline constexpr std::size_t MetadataCap=1024u<<10;
struct Profile {
    bool type45=false;
    bool structural_limit=false;
    bool beam18=false;
};
bool SameProfile(Profile,Profile) noexcept;
Document ProfileDocument(Profile);
Profile ReadProfile(const Value&);
struct Values {
    std::uint64_t owner=0;
    records::FrameStamp stamp;
    std::optional<double> structural_limit_s;
};
struct Sequence {
    records::FrameStamp last;
};
void CheckValues(const records::Context&,Profile,const Values&);
Sequence Advance(const records::Context&,Profile,std::uint64_t planned,const Sequence&,const Values&);
std::vector<std::string> IntegerFields();
std::vector<std::string> RealFields(Profile);
struct Segment {
    std::uint64_t first_epoch=0,rows=0;
    arrays::Descriptor integers,reals;
};
struct FrameFiles {
    records::FrameStamp stamp;
    records::RecordFile frame,activity;
};
struct Index {
    std::uint64_t planned_intervals=0,accepted_intervals=0;
    bool horizon_complete=false;
    std::string stop_reason;
    records::FrameStamp final;
    std::vector<Segment> segments;
    std::vector<FrameFiles> frames;
};
Document StampDocument(const records::FrameStamp&);
records::FrameStamp ReadStamp(const Value&);
Document FileDocument(const records::RecordFile&);
records::RecordFile ReadFileRecord(const Value&);
std::string ReadFile(const std::filesystem::path&,const records::RecordFile&,std::size_t cap);
records::RecordFile WriteDocument(const std::filesystem::path&,const std::string&,const Document&,std::size_t cap);
} // namespace crash::output::physical_run
