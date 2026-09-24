#pragma once
#include "Types.h"
#include <functional>
#include <memory>
namespace crash::output::physical_run {
// Formatting layer. Live callers obtain Values from the accepted dynamics
// factory, never from a candidate observation or a reconstructed timestamp.
class IntervalWriter {
  public:
    IntervalWriter(std::filesystem::path,const records::Context&,Profile,std::uint64_t planned,
        std::size_t file_cap,std::size_t host_cap);
    ~IntervalWriter();
    void Append(const Values&);
    std::vector<Segment> Finish();
    const Sequence& sequence() const noexcept;
    std::size_t buffer_bytes() const noexcept;
    bool failed() const noexcept;
  private:
    struct Data;
    std::unique_ptr<Data> data_;
    void Flush();
};
// Exact conservative staging reservation used by ReadIntervals. It follows the
// declared chunk/planned horizon and observation column shape, even for a short
// accepted prefix. It excludes retained source/frame replay storage.
std::size_t IntervalReadStagingBytes(Profile,std::uint64_t planned,std::size_t file_cap);
Sequence ReadIntervals(const std::filesystem::path&,const records::Context&,Profile,
    std::uint64_t planned,std::uint64_t accepted,const std::vector<Segment>&,
    std::size_t file_cap,std::size_t host_cap,const std::function<void(const Values&)>&);
} // namespace crash::output::physical_run
