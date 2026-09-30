#pragma once
#include "SourcePartElasticInternal.h"
#include "SourcePartElasticPilot.h"
#include "lib_utest/qualification/qeph/QephForceFixture.h"
#include "lib_utest/qualification/t3/T3ForcePortFixture.h"

namespace crash::cases::source_part_elastic {
// Qualification-only access; no public case API accepts fabricated candidates.
struct SourcePartElasticTestAccess {
    static auto& Internal(SourcePartElasticCase& c) { return *c.impl_; }
};
namespace test {
const std::filesystem::path& SourceReadinessPath();
namespace qn=tl::qualification::qeph;
namespace tn=tl::qualification::t3;
struct NativeSequence {
    std::array<qn::Reference,source::Q4Count> qr;
    std::array<tn::Reference,source::T3Count> tr;
    std::array<qn::History,source::Q4Count> qhistory;
    std::array<tn::History,source::T3Count> thistory;
    std::array<qn::ForceTrial,source::Q4Count> qtrial;
    std::array<tn::ForceTrial,source::T3Count> ttrial;
    // Test-only prior CUDA endpoint histories, retained without another device
    // read. They are actual accepted bases on the next sequential Check call.
    std::array<tn::HistoryValues,source::T3Count> previous_cuda_t3;
    std::array<tn::HistoryStamp,source::T3Count> previous_cuda_t3_stamp;
    std::array<bool,source::T3Count> previous_cuda_t3_available{};
    bool printed_t3_failure=false;
    void Initialize(SourcePartElasticCase&);
    void Check(SourcePartElasticCase&,const Snapshot& base,const Snapshot& endpoint,
        const std::array<long double,3*NodeCount>* additional_base_force=nullptr);
    void Accept();
};
struct Results {
    std::array<q::ForceTrial,source::Q4Count> qeph;
    std::array<t::ForceTrial,source::T3Count> t3;
};
void ReadResults(SourcePartElasticCase&,Results&);
void SameResults(const Results&,const Results&);
void SameSnapshot(const Snapshot&,const Snapshot&,bool same_owner=true);
} // namespace test
} // namespace crash::cases::source_part_elastic
