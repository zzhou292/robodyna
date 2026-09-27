#include "ShellMixedSectionStorage.h"
#include "one_point/ShellOnePointStorage.h"
#include "failure/ShellFailureStorage.h"
#include "ShellResidentHostAccounting.h"
#include <new>
#include <stdexcept>
#include <utility>

namespace tl::fea::shell_batch_plasticity_detail {
MixedHostStorage::~MixedHostStorage() { if(device_)cudaFree(device_); }
bool MixedHostStorage::Forecast(std::size_t count,std::size_t points,std::size_t catalog_bytes,
    std::size_t device_cap,std::size_t host_cap,MixedLayout& output,std::size_t& host_bytes,bool global) noexcept {
  MixedLayout layout;
  if(!host_cap||host_cap>MaxVehicleShellResidentHostBytes||!layout.Initialize(count,points,device_cap,global))return false;
  util::BoundedArenaLayout host(host_cap);util::ArenaRegion ignored;
  if(!host.Append<unsigned char>(sizeof(HostStorage)+sizeof(MixedHostStorage),ignored)||
     !host.Append<unsigned char>(layout.bytes,ignored)||!host.Append<ShellBatchSectionState>(count,ignored)||
     !host.Append<sections::ShellLayeredLaw1History>(count,ignored)||
     !host.Append<ShellBatchLayeredSection>(count,ignored)||!host.Append<std::size_t>(count,ignored)||
     !host.Append<unsigned char>(catalog_bytes,ignored)||!host.Append<unsigned char>(3*64,ignored))return false;
  output=layout;host_bytes=host.bytes();return true;
}
bool HostStorage::ForecastSections(std::size_t count,std::size_t points,std::size_t catalog_bytes,
    std::size_t device_cap,std::size_t host_cap,std::size_t& host_bytes) noexcept {
  MixedLayout layout;return MixedHostStorage::Forecast(count,points,catalog_bytes,device_cap,host_cap,layout,host_bytes);
}
std::size_t HostStorage::device_bytes() const noexcept {
  return (mixed_?mixed_->device_bytes():device_?layout_.bytes:0)+failure_device_bytes()+
      (one_point_?one_point_->device_bytes():0);
}
MixedDeviceStorage* HostStorage::mixed_device() const noexcept { return mixed_?mixed_->device():nullptr; }
OnePointDeviceStorage* HostStorage::one_point_device() const noexcept {
  return one_point_?one_point_->device():nullptr;
}
SetupReport HostStorage::ReadSections(unsigned slab,std::size_t count,cudaStream_t stream,double time) noexcept {
  const auto report=ReadSectionsBeforeFailure(slab,count,stream,time);
  return report.status==SetupStatus::Success&&failure_?
    failure_->Read(slab,count,stream,time,mixed_->staging()):report;
}
SetupReport HostStorage::ReadActivitySections(unsigned slab,std::size_t count,cudaStream_t stream,double time) noexcept {
  const auto report=ReadSectionsBeforeFailure(slab,count,stream,time);
  if(report.status!=SetupStatus::Success)return report;
  if(!failure_)return {SetupStatus::InvalidInput,"Invalid failure readback shape"};
  return failure_->CheckReadSources(slab,count,mixed_->staging());
}
SetupReport HostStorage::CheckActivitySectionSources(unsigned slab,std::size_t count) const noexcept {
  const auto* catalog=Collection();
  if(!mixed_||!catalog)return {SetupStatus::InvalidInput,"No explicit mixed section history"};
  if(one_point_)return {SetupStatus::InvalidInput,"One-point history is unavailable"};
  return mixed_->CheckActivitySources(slab,count,*catalog);
}
SetupReport HostStorage::CheckActivityFailureSources(unsigned slab,std::size_t count) const noexcept {
  if(!failure_)return {SetupStatus::InvalidInput,"Invalid failure readback shape"};
  return failure_->CheckActivitySources(slab,count);
}
bool HostStorage::SupportsCompactActivity(unsigned slab,std::size_t count,ShellBindingFamily family) const noexcept {
  const auto* catalog=Collection();
  if(!mixed_||!failure_||!catalog||count!=element_count_||mixed_->family()!=family||
      !mixed_->HasReadShape(slab,count,*catalog)||
      (one_point_&&!one_point_->HasReadShape(slab,count))||
      failure_->CheckActivitySources(slab,count).status!=SetupStatus::Success) return false;
  for(std::size_t parent=0;parent<count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if(!catalog->Law(family,parent,&law)) return false;
    if(law==ShellSectionLaw::Law44Nip1) {
      sections::PointParameters parameters;
      if(family!=ShellBindingFamily::T3||!one_point_||!catalog->Parameters(family,parent,&parameters)) return false;
    } else if(law!=ShellSectionLaw::LayeredLaw1Nip3&&law!=ShellSectionLaw::LayeredLaw44Nip3&&
        ((law!=ShellSectionLaw::RigidSkin&&law!=ShellSectionLaw::GlobalLaw1Npt0)||!catalog->execution_sections())) return false;
  }
  return true;
}
SetupReport HostStorage::ReadSectionsBeforeFailure(unsigned slab,std::size_t count,cudaStream_t stream,double time) noexcept {
  const auto* catalog=Collection();
  if(!mixed_||!catalog)return {SetupStatus::InvalidInput,"No explicit mixed section history"};
  if(one_point_) {
    const auto report=one_point_->Read(slab,count,stream,*catalog,time);
    if(report.status!=SetupStatus::Success) return report;
  }
  return mixed_->Read(slab,count,stream,*catalog,one_point_?one_point_->staging():nullptr);
}
const ShellBatchLayeredSection* HostStorage::section_staging() const noexcept {
  return mixed_?mixed_->staging():nullptr;
}
SetupReport HostStorage::InitializeSections(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchBinding& binding,ShellBindingFamily family,std::size_t count,
    std::size_t device_cap,std::size_t host_cap,bool vehicle) try {
  ShellSectionCounts counts;
  if(!catalog.Counts(family,&counts)||counts.law44_nip1)
    return {SetupStatus::InvalidInput,"One-point T3 requires complete constant failure binding"};
  std::size_t binding_bytes=0,catalog_bytes=0,host_bytes=0;MixedLayout layout;
  if(!shell_batch_detail::RetainedScopeBytes(&binding,&catalog,vehicle,binding_bytes,catalog_bytes)||
     !MixedHostStorage::Forecast(count,catalog.curve_point_count(),catalog_bytes,device_cap,host_cap,layout,host_bytes,counts.law1_global_npt0!=0))
    return {SetupStatus::ResourceLimit,"Mixed layered section exceeds active byte budgets"};
  auto owned=std::unique_ptr<ShellBatchPlasticityBinding>(new(std::nothrow) ShellBatchPlasticityBinding(catalog));
  auto next=std::unique_ptr<MixedHostStorage>(new(std::nothrow) MixedHostStorage);
  if(!owned||!next)return {SetupStatus::ResourceLimit,"Mixed layered section host allocation failed"};
  const auto report=next->Initialize(*owned,family,count,layout);
  if(report.status!=SetupStatus::Success)return report;
  collection_=std::move(owned);mixed_=std::move(next);element_count_=count;
  return {SetupStatus::Success,"OK"};
} catch(const std::bad_alloc&) { return {SetupStatus::ResourceLimit,"Mixed section host allocation failed"}; }
  catch(const std::length_error&) { return {SetupStatus::ResourceLimit,"Mixed section host size overflow"}; }

SetupReport MixedHostStorage::Initialize(const ShellBatchPlasticityBinding& catalog,
    ShellBindingFamily family,std::size_t count,const MixedLayout& layout,bool execution) {
  if(device_||!catalog.heterogeneous_sections()||!count||count!=layout.law.count||
      catalog.execution_sections()!=execution||
      (family!=ShellBindingFamily::Qeph&&family!=ShellBindingFamily::T3))
    return {SetupStatus::InvalidInput,"Invalid explicit mixed section layout"};
  ShellSectionCounts counts;
  if(!catalog.Counts(family,&counts)||
      layout.global_profiles.count!=(counts.law1_global_npt0?count:0))
    return {SetupStatus::InvalidInput,"Global LAW1 profile storage differs from the complete catalog"};
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes))return {SetupStatus::ResourceLimit,"Mixed section staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial)return {SetupStatus::ResourceLimit,"Mixed section startup layout is invalid"};
  plastic_.Resize(count);elastic_.Resize(count);output_.Resize(count);
  const auto& source=catalog.data_;
  for(std::size_t i=0;i<source.point_count;++i) {
    initial->plastic.curve_x[i]=source.curve_x[i];initial->plastic.curve_y[i]=source.curve_y[i];
  }
  auto offsets=std::unique_ptr<std::size_t[]>(new(std::nothrow) std::size_t[count]{});
  if(!offsets)return {SetupStatus::ResourceLimit,"Mixed section curve-offset allocation failed"};
  for(std::size_t e=0;e<count;++e) {
    offsets[e]=NoShellBindingNode;
    if(!catalog.Law(family,e,&initial->law[e]))
      return {SetupStatus::InvalidInput,"Mixed catalog does not resolve a native parent"};
    if(initial->law[e]==ShellSectionLaw::GlobalLaw1Npt0) {
      if(!execution||!initial->global_law1||!catalog.GlobalLaw1Profile(family,e,&initial->global_law1[e]))
        return {SetupStatus::InvalidInput,"Global LAW1 immutable profile is unavailable"};
    } else if(initial->law[e]==ShellSectionLaw::LayeredLaw1Nip3) {
      if(!catalog.ElasticParameters(family,e,&initial->elastic_parameters[e]))
        return {SetupStatus::InvalidInput,"Mixed catalog elastic parameters are unavailable"};
    } else if(initial->law[e]==ShellSectionLaw::LayeredLaw44Nip3||initial->law[e]==ShellSectionLaw::Law44Nip1) {
      auto& p=initial->plastic.parameters[e];
      if(!catalog.Parameters(family,e,&p))return {SetupStatus::InvalidInput,"Mixed catalog plastic parameters are unavailable"};
      if(p.hardening==material::ShellPlasticityHardeningKind::LinearLaw44)continue;
      offsets[e]=static_cast<std::size_t>(p.curve.plastic_strain-source.curve_x.data());
      if(offsets[e]>source.point_count||p.curve.count>source.point_count-offsets[e])
        return {SetupStatus::InvalidInput,"Mixed material curve exceeds the owned pool"};
    } else if(initial->law[e]!=ShellSectionLaw::RigidSkin||!execution) {
      return {SetupStatus::InvalidInput,"Mixed catalog has an unsupported law"};
    }
  }
  MixedDeviceStorage* candidate=nullptr;auto error=cudaMalloc(reinterpret_cast<void**>(&candidate),layout.bytes);
  if(error!=cudaSuccess)return {SetupStatus::DeviceFailure,"Mixed section allocation failed",error};
  const auto header=layout.Rebase(*initial,candidate);
  for(std::size_t e=0;e<count;++e)if(offsets[e]!=NoShellBindingNode) {
    initial->plastic.parameters[e].curve.plastic_strain=header.plastic.curve_x+offsets[e];
    initial->plastic.parameters[e].curve.yield_stress_pa=header.plastic.curve_y+offsets[e];
  }
  *initial=header;error=cudaMemcpy(candidate,arena.data(),layout.bytes,cudaMemcpyHostToDevice);
  if(error!=cudaSuccess) {
    cudaFree(candidate);return {SetupStatus::DeviceFailure,"Mixed section initialization copy failed",error};
  }
  device_=candidate;header_=header;layout_=layout;count_=count;family_=family;
  return {SetupStatus::Success,"OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
