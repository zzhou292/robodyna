#pragma once
#include "case/source_assembly_observation/SourceAssemblyQephSpin.h"
#include "output/ArtifactIO.h"
#include <memory>

namespace crash::cases::source_assembly_dynamics { class SourceAssemblyWallCase; }
namespace crash::output::assembly {
inline constexpr std::size_t SpinTraceRowCap=64*1024,SpinTraceByteCap=256*1024*1024;
std::size_t PlanQephSpinTrace(std::uint64_t steps,std::uint64_t cadence);
Document QephSpinDocument(const cases::source_assembly_observation::QephSpinObservation&);
// Optional create-only companion, outside the accepted archive. No allocations
// inside case Step: serialization and checked file writes happen after commit.
// Complete/prefix footer is last; missing footer means incomplete trace.
class QephSpinTrace {
  public:
    QephSpinTrace(const std::filesystem::path&,const cases::source_assembly_dynamics::SourceAssemblyWallCase&,
                  std::uint64_t steps,std::uint64_t cadence);
    ~QephSpinTrace();
    QephSpinTrace(const QephSpinTrace&)=delete;
    QephSpinTrace& operator=(const QephSpinTrace&)=delete;
    void RecordInterval(const cases::source_assembly_dynamics::SourceAssemblyWallCase&);
    void Finish(const cases::source_assembly_dynamics::SourceAssemblyWallCase&,bool horizon_complete,const std::string& reason);
  private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
}
