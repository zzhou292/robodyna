#include "JsonReader.h"
namespace crash::modelio::assembly::reader {
void ReadMaterialPolicy(const Value& document,const Data& data) {
    if(data.schema==InventorySchema)return;
    const auto& law=Member(document,"law44_policy");
    TextIs(law,"revision","a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(law,"hardening_model","law44_linear");TextIs(law,"A","SIGY");TextIs(law,"B","ETAN*E/(E-ETAN)");
    Require(Unsigned(law,"n")==1&&Unsigned(law,"function_reference")==0&&Unsigned(law,"vp")==0,
            "Unsupported analytic LAW44 conversion policy");
    Same(Real(law,"source_time_to_s"),1);Same(Real(law,"rate_filter_hz"),10000);
    Flag(law,"case_integration_qualified",false);
    if(data.schema!=SectionInventorySchema)return;
    const auto& policy=Member(document,"section_policy");TextIs(policy,"policy","layered_law1_or_law44");
    const auto& elastic=Member(policy,"law1");
    TextIs(elastic,"revision","a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(elastic,"material_source","reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:218-239");
    TextIs(elastic,"section_source","reader/source/dyna2rad/dyna2rad/_private/convertprops.cxx:776-813");
    TextIs(elastic,"material_law","layered_law1");
    Require(Unsigned(elastic,"NIP")==3&&Unsigned(elastic,"ITHICK")==1,"Unsupported elastic section policy");
    Flag(elastic,"case_integration_qualified",false);
}
} // namespace crash::modelio::assembly::reader
