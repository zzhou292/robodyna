#include "ReferenceSourceSupport.h"
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include <unordered_map>

namespace crash::cases::vehicle_startup::test {
TEST(VehicleReferenceSource, ActualV3ProjectionsMatchExistingCompleteBindingNativeValues) {
    const auto& r=References();std::unordered_map<std::uint64_t,std::size_t> rows;
    rows.reserve(r.rows().size());for(std::size_t i=0;i<r.rows().size();++i)rows.emplace(r.rows()[i].element_id,i);
    for(bool mixed:{false,true}) {
        const auto* path=std::getenv(mixed?"ROBO_VEHICLE_MIXED":"ROBO_VEHICLE_ELASTIC");ASSERT_NE(path,nullptr);
        const auto original=modelio::assembly::SourceAssembly::Read(path,modelio::assembly::test::section::Identity(mixed));
        const modelio::assembly::SourceAssemblyShellInput input(original);
        tl::fea::ShellBatchBinding binding;
        ASSERT_EQ(binding.Initialize(input.input(),tl::fea::ShellHostBindingLimits{}).status,tl::fea::ShellBindingStatus::Success);
        EXPECT_EQ(original.data().parents.size(),mixed?631:149);
        for(const auto& p:original.data().parents) {
            const auto found=rows.find(p.source_id);ASSERT_NE(found,rows.end());const auto i=found->second;
            ASSERT_EQ(r.rows()[i].part_id,p.part_id);
            if(p.family==modelio::assembly::ShellFamily::Qeph) {
                ASSERT_NE(r.qeph(i),nullptr);Same(*r.qeph(i),binding.qeph_reference(p.family_index));
                SameInput(r.qeph(i)->input,input.input().qeph[p.family_index].reference);
            } else {
                ASSERT_NE(r.t3(i),nullptr);Same(*r.t3(i),binding.t3_reference(p.family_index));
                SameInput(r.t3(i)->input,input.input().t3[p.family_index].reference);
            }
        }
    }
}
} // namespace crash::cases::vehicle_startup::test
