#pragma once
#include "SourceAssemblyWallSchema.h"
#include <memory>

namespace crash::cases::source_assembly_dynamics { class SourceAssemblyWallCase; }
namespace tl::fea { struct NodalStamp; }
namespace crash::output::assembly {
// Serial accepted-output writer. The live case outlives calls and is externally
// serialized with stepping. Only accepted captures from this exact case enter
// the archive; the writer owns no mechanical state, material history or clock.
// Exceptions leave partial files without a completed manifest. Call Fail to
// record a reason. A deliberate accepted prefix closes only its actual rows.
class SourceAssemblyWallArtifacts {
  public:
    SourceAssemblyWallArtifacts(const std::string& new_directory,
        cases::source_assembly_dynamics::SourceAssemblyWallCase&,const WallArchiveRequest&);
    ~SourceAssemblyWallArtifacts();
    SourceAssemblyWallArtifacts(const SourceAssemblyWallArtifacts&)=delete;
    SourceAssemblyWallArtifacts& operator=(const SourceAssemblyWallArtifacts&)=delete;
    void RecordInterval(const tl::fea::NodalStamp& base,cases::source_assembly_dynamics::SourceAssemblyWallCase&);
    void WriteFrame(cases::source_assembly_dynamics::SourceAssemblyWallCase&);
    void Finish(cases::source_assembly_dynamics::SourceAssemblyWallCase&,double elapsed_seconds);
    void FinishPrefix(cases::source_assembly_dynamics::SourceAssemblyWallCase&,double elapsed_seconds,const std::string& reason);
    void Fail(const std::string&) noexcept;
  private:
    void Close(cases::source_assembly_dynamics::SourceAssemblyWallCase&,double,const std::string&,bool prefix);
    struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace crash::output::assembly
