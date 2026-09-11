#pragma once
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "output/full_shell/FullShellVisualizationPlan.h"

namespace crash::output::full_shell::activity {
inline constexpr const char* Schema="robo_dyna.full_shell_parent_activity.v1";
inline constexpr const char* DeclarationSchema="robo_dyna.full_shell_parent_activity_declaration.v1";
inline constexpr std::size_t MetadataByteCap=FrameMetadataByteCap;
struct Limits {std::size_t host_bytes=64*1024*1024;};
namespace detail {std::size_t Preflight(const Context&,Limits);}
struct ActivityInput {
    Identity identity;
    std::string point_layout_sha256;
    FrameStamp stamp;
    const std::uint8_t* parent_active=nullptr;
    std::size_t parents=0;
};
// Lossless accepted parent mechanics only; point failure and contact eligibility
// are separate channels. Caller supplies the actual accepted scope/stamp. Value
// association checks do not authenticate a live solver or create acceptance.
class ActivityRecord {
  public:
    static ActivityRecord Create(const Context&,const ActivityInput&,const FrameStamp& expected,Limits={});
    ActivityRecord(const ActivityRecord&) noexcept=default;
    ActivityRecord(ActivityRecord&& other) noexcept:data_(other.data_) {}
    ActivityRecord& operator=(const ActivityRecord&)=delete;
    ActivityRecord& operator=(ActivityRecord&&)=delete;
    const Context& context() const noexcept;
    const FrameStamp& stamp() const noexcept;
    const std::vector<std::uint64_t>& words() const noexcept;
    std::size_t active_count() const noexcept;
    bool active(std::size_t source_parent_index) const;
  private:
    friend std::size_t detail::Preflight(const Context&,Limits);
    friend ActivityRecord ReadActivity(const std::filesystem::path&,const Context&,const RecordFile&,const FrameStamp&,Limits);
    struct Data;
    static ActivityRecord FromWords(const Context&,FrameStamp,std::vector<std::uint64_t>);
    explicit ActivityRecord(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
// Two create-only files: <stem>.activity.bin and <stem>.activity.json, metadata
// last. All destinations/serialization preflight precedes the first write.
// I/O failure can leave incomplete evidence; only a later run publisher owns a
// completion manifest. Calls are externally serialized like ArtifactIO.
RecordFile WriteActivity(const std::filesystem::path&,const std::string& stem,const ActivityRecord&);
ActivityRecord ReadActivity(const std::filesystem::path&,const Context&,const RecordFile&,const FrameStamp& expected,Limits={});
RecordFile WriteDeclaration(const std::filesystem::path&,const std::string& file,const Context&,Limits={});
void ReadDeclaration(const std::filesystem::path&,const Context&,const RecordFile&,Limits={});
struct ActivityPlan {
    Plan archive;
    std::size_t packed_frame_bytes=0,frame_metadata_bytes=MetadataByteCap,declaration_bytes=MetadataByteCap;
};
// Sidecar consumes explicit static reserve; both activity files are charged for
// every saved frame and the accepted-prefix reserve. Other extra-frame records
// require a later combined profile. V1 PlanArchive is unchanged.
ActivityPlan PlanWithActivity(const Context&,const PlanRequest&,const std::string& declaration_file);
} // namespace crash::output::full_shell::activity
