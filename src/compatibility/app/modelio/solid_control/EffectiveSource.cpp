#include "Internal.h"
#include "Storage.h"
namespace crash::modelio::solid_control {
struct EffectiveSource::Data {
    Data(const DirectSource& value, const ids::ImportContext& context) : direct(value), imported(context) {}
    DirectSource direct;
    ids::ImportContext imported;
    EffectiveData values;
    std::size_t owned_bytes = 0;
};
Preparation EffectiveSource::Prepare(const DirectSource& direct, const ids::ImportContext& imported,
                                    const ids::ImportMembers& members, Limits limits) {
    Preparation result;
    try {
        const Limits hard;
        if (!limits.parts || limits.parts > hard.parts || !limits.source_blocks ||
            limits.source_blocks > hard.source_blocks || !limits.metadata_bytes || limits.metadata_bytes > hard.metadata_bytes || !limits.retained_bytes || limits.retained_bytes > hard.retained_bytes)
            detail::Reject(Status::ResourceLimit, "Invalid effective solid-control capacity");
        auto next = std::make_shared<Data>(direct, imported);
        next->values = detail::ReadEffective(direct, imported, members, limits);
        next->owned_bytes = detail::EffectiveBytes(next->values, sizeof(Data));
        if (next->owned_bytes > limits.retained_bytes)
            detail::Reject(Status::ResourceLimit, "Effective solid-control retained result exceeds reservation");
        result.source.emplace(EffectiveSource(std::move(next)));
        result.report.status = Status::Ready;
    } catch (const detail::Failure& error) { result.report = error.report; }
    catch (const std::bad_alloc&) { result.report = {Status::ResourceLimit, "Effective solid-control allocation failed", {}, 0}; }
    catch (const std::exception& error) { result.report = {Status::InvalidInput, std::string(error.what()).substr(0,1024), {}, 0}; }
    return result;
}
const DirectSource& EffectiveSource::direct() const noexcept { return data_->direct; }
const EffectiveData& EffectiveSource::data() const noexcept { return data_->values; }
std::size_t EffectiveSource::owned_payload_bytes() const noexcept { return data_->owned_bytes; }
} // namespace crash::modelio::solid_control
