#pragma once
#include "modelio/native_contact_scene/DeclaredSource.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_src/constraints/tied_shell/search/TiedSearchFinalization.h"
namespace crash::cases::native_scene {
struct TiedSourceLimits {std::size_t host_bytes=8u<<20;};
// Immutable named-source value composition only. No current coefficients,
// witness activity, physical clock or native Transaction admission is exposed.
class TiedSource {
  public:
    static TiedSource Prepare(const modelio::native_scene::DeclaredSource&,
        const tl::fea::NodalNodeDomain&,TiedSourceLimits={});
    const tl::constraints::tied_shell::TiedCinAttachmentModel& model() const noexcept;
    const tl::constraints::tied_shell::FinalizedSearch& search() const noexcept;
    const tl::constraints::tied_shell::ClassificationResult& classification() const noexcept;
    const tl::constraints::tied_shell::PostKinChkResult& post_kinchk() const noexcept;
    const std::array<tl::constraints::tied_shell::WorkingSearchInput,4>& search_inputs() const noexcept;
    std::size_t retained_host_upper_bound() const noexcept;
  private:
    struct Data;
    explicit TiedSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
}
