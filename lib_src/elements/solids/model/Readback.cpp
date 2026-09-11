// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Impl.h"
#include "../../solid18/Solid18ForceChecks.h"
#include "../../solid24/Solid24ForceHistory.h"
#include "../../solid6z/Solid6zForceChecks.h"
#include "../../solid18/law44/ForceChecks.h"
#include "../../solid18/total_strain/ForceChecks.h"

namespace tl::fea::solids {
ModelProfile Model::profile() const noexcept {
  return impl_ ? impl_->profile : ModelProfile::OriginalThreeFamilies;
}
std::uint64_t Model::source_instance_id() const noexcept {
  return domain()?domain()->source_instance_id():0;
}
const NodalNodeDomain* Model::domain() const noexcept {
  return impl_?impl_->coefficients.domain():nullptr;
}
const SolidNodeContributions* Model::contributions() const noexcept {
  return impl_?&impl_->coefficients:nullptr;
}
util::ConstView<Parent18> Model::solid18() const noexcept {
  static const Parent18 empty;
  return {impl_?impl_->storage.parent18:&empty,impl_?impl_->layout.parent18.count:0};
}
util::ConstView<Parent24> Model::solid24() const noexcept {
  static const Parent24 empty;
  return {impl_?impl_->storage.parent24:&empty,impl_?impl_->layout.parent24.count:0};
}
util::ConstView<Parent6z> Model::solid6z() const noexcept {
  static const Parent6z empty;
  return {impl_?impl_->storage.parent6z:&empty,impl_?impl_->layout.parent6z.count:0};
}
util::ConstView<Material36> Model::materials36() const noexcept {
  static const Material36 empty;
  return {impl_?impl_->storage.material36:&empty,impl_?impl_->layout.material36.count:0};
}
util::ConstView<Material42> Model::materials42() const noexcept {
  static const Material42 empty;
  return {impl_?impl_->storage.material42:&empty,impl_?impl_->layout.material42.count:0};
}

util::ConstView<Parent18Law44> Model::solid18_law44() const noexcept {
  static const Parent18Law44 empty;
  return {impl_ ? impl_->storage.parent44 : &empty, impl_ ? impl_->layout.parent44.count : 0};
}
util::ConstView<Parent18Law90> Model::solid18_law90() const noexcept {
  static const Parent18Law90 empty;
  return {impl_ ? impl_->storage.parent90 : &empty, impl_ ? impl_->layout.parent90.count : 0};
}
util::ConstView<Material44> Model::materials44() const noexcept {
  static const Material44 empty;
  return {impl_ ? impl_->storage.material44 : &empty, impl_ ? impl_->layout.material44.count : 0};
}
util::ConstView<Material90> Model::materials90() const noexcept {
  static const Material90 empty;
  return {impl_ ? impl_->storage.material90 : &empty, impl_ ? impl_->layout.material90.count : 0};
}
bool Model::SharesStorage(const Model& other) const noexcept {
  return impl_ && impl_==other.impl_;
}
bool Model::Matches(const Model& other) const noexcept {
  if(!impl_ || !other.impl_)return false;
  if(SharesStorage(other))return true;
  if(profile()!=other.profile() || !contributions()->Matches(*other.contributions()) ||
      solid18().size()!=other.solid18().size() || solid24().size()!=other.solid24().size() ||
      solid6z().size()!=other.solid6z().size() || materials36().size()!=other.materials36().size() ||
      materials42().size()!=other.materials42().size() || materials44().size()!=other.materials44().size() ||
      materials90().size()!=other.materials90().size() || solid18_law44().size()!=other.solid18_law44().size() ||
      solid18_law90().size()!=other.solid18_law90().size())return false;
  for(std::size_t i=0;i<materials36().size();++i) {
    const auto& a=materials36()[i];const auto& b=other.materials36()[i];
    if(a.source_material_id!=b.source_material_id || !model_detail::Same(a.value,b.value))return false;
  }
  for(std::size_t i=0;i<materials42().size();++i) {
    const auto& a=materials42()[i];const auto& b=other.materials42()[i];
    if(a.source_material_id!=b.source_material_id || !model_detail::Same(a.value,b.value))return false;
  }
  for(std::size_t i=0;i<solid18().size();++i) {
    const auto& a=solid18()[i];const auto& b=other.solid18()[i];
    if(a.material_index!=b.material_index || !solid18::detail::SameReference(a.reference,b.reference))return false;
  }
  for(std::size_t i=0;i<solid24().size();++i) {
    const auto& a=solid24()[i];const auto& b=other.solid24()[i];
    if(a.material_index!=b.material_index || !solid24::force_detail::SameReference(a.reference,b.reference))return false;
  }
  for(std::size_t i=0;i<solid6z().size();++i) {
    const auto& a=solid6z()[i];const auto& b=other.solid6z()[i];
    if(a.material_index!=b.material_index || !model_detail::Same(a.profile,b.profile) ||
        !solid6z::force_detail::Same(a.reference,b.reference))return false;
  }
  for (std::size_t i=0;i<materials44().size();++i) {
    const auto& a=materials44()[i]; const auto& b=other.materials44()[i];
    if (a.source_material_id!=b.source_material_id || !model_detail::Same(a.value,b.value)) return false;
  }
  for (std::size_t i=0;i<materials90().size();++i) {
    const auto& a=materials90()[i]; const auto& b=other.materials90()[i];
    if (a.source_material_id!=b.source_material_id || !model_detail::Same(a.value,b.value)) return false;
  }
  for (std::size_t i=0;i<solid18_law44().size();++i) {
    const auto& a=solid18_law44()[i]; const auto& b=other.solid18_law44()[i];
    if (a.material_index!=b.material_index || !solid18::law44::detail::SameReference(a.reference,b.reference)) return false;
  }
  for (std::size_t i=0;i<solid18_law90().size();++i) {
    const auto& a=solid18_law90()[i]; const auto& b=other.solid18_law90()[i];
    if (a.material_index!=b.material_index || !solid18::total_strain::force_detail::SameReference(a.reference,b.reference)) return false;
  }
  return true; // Domain/source-slot bit identity was checked by the contribution snapshot.
}
std::size_t Model::owned_payload_bytes() const noexcept {
  return impl_?impl_->layout.owned_bytes:sizeof(*this);
}
std::size_t Model::startup_payload_bytes() const noexcept {
  return impl_?impl_->layout.startup_bytes:sizeof(*this);
}
} // namespace tl::fea::solids
