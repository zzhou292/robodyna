#pragma once
#include "Archive.h"
namespace crash::output::physical_frames {
struct Archive::Data {
    Data(const records::Context& c,source::PreparedSourceBundle b,records::activity::ActivityPlan p)
        :context(c),bundle(std::move(b)),plan(std::move(p)) {}
    records::Context context;
    source::PreparedSourceBundle bundle;
    records::activity::ActivityPlan plan;
    std::size_t host_bytes=0;
    std::uint64_t intervals=0;
    std::vector<std::uint64_t> written;
    bool prefix_used=false,io_failed=false;
};
namespace detail {
void FrameDestinations(const std::filesystem::path&,const std::string&);
void CheckPair(const records::Context&,const records::FrameRecord&,
    const records::activity::ActivityRecord&);
} // namespace detail
} // namespace crash::output::physical_frames
