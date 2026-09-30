#pragma once
#include "source_part_elastic/SourcePartElasticCase.h"
#include <memory>
#include <string>
namespace crash::cases::source_part_wall {
// Accepted data only. A declared horizon or explicitly stopped accepted prefix
// can be closed for replay; neither claims rebound, crash accuracy or restart.
class SourcePartWallArtifacts {
  public:
    SourcePartWallArtifacts(const std::string& new_directory,source_part_elastic::SourcePartElasticCase&,
        std::uint64_t requested_steps,unsigned frame_every,std::uint64_t run_id,std::uint64_t topology_id);
    ~SourcePartWallArtifacts();
    void RecordInterval(const tl::fea::NodalStamp& base,source_part_elastic::SourcePartElasticCase&);
    void WriteFrame(source_part_elastic::SourcePartElasticCase&);
    void Finish(source_part_elastic::SourcePartElasticCase&,double elapsed_seconds);
    void FinishPrefix(source_part_elastic::SourcePartElasticCase&,double elapsed_seconds,const std::string& stop_reason);
    void Fail(const std::string&) noexcept;
  private:
    void Close(source_part_elastic::SourcePartElasticCase&,double,const std::string&,bool prefix);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
