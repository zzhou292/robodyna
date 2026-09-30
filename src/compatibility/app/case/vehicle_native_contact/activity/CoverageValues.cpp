#include "Coverage.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact::activity::coverage {
void CheckRecord(const std::vector<std::uint64_t>& records,std::size_t columns,std::size_t row,
    std::uint64_t id,const std::uint64_t* nodes,unsigned arity,std::vector<std::uint8_t>& selected) {
    output::Require(columns>=2&&arity<=columns-2&&records.size()%columns==0&&
        selected.size()==records.size()/columns&&row<selected.size()&&nodes,
        "Activity source record shape is incomplete");
    output::Require(!selected[row]&&id&&records[row*columns]==id,
        "Activity support is missing, duplicated or mapped to the wrong original element");
    for(unsigned k=0;k<arity;++k)output::Require(nodes[k]&&records[row*columns+2+k]==nodes[k],
        "Activity source ordered physical endpoint differs from its canonical record");
    selected[row]=1;
}
Population Finish(const std::vector<std::uint64_t>& records,std::size_t columns,unsigned arity,
    const std::vector<std::uint8_t>& selected,const tl::fea::NodalNodeDomain& domain) {
    output::Require(columns>=2&&arity<=columns-2&&records.size()%columns==0&&selected.size()==records.size()/columns,
        "Activity source exclusion census shape differs");
    Population out;out.original=selected.size();std::vector<std::uint64_t> executed,excluded;
    executed.reserve(out.original);excluded.reserve(out.original);
    for(std::size_t row=0;row<out.original;++row) {
        output::Require(selected[row]<=1,"Activity source inclusion flag is invalid");
        if(selected[row]){executed.push_back(records[row*columns]);++out.executed;continue;}
        excluded.push_back(records[row*columns]);++out.excluded;bool touches=false;
        for(unsigned k=0;k<arity;++k)touches=touches||domain.Find(records[row*columns+2+k])!=SIZE_MAX;
        out.excluded_touching_owner+=touches;
    }
    auto digest=[](std::vector<std::uint64_t>& ids) {
        std::sort(ids.begin(),ids.end());
        output::Require(std::adjacent_find(ids.begin(),ids.end())==ids.end(),"Repeated source element identity in activity coverage");
        return output::Sha256(output::arrays::Encode({output::arrays::Scalar::UInt64,ids.size(),1,{}},ids.data(),ids.size()));
    };
    out.executed_ids_sha256=digest(executed);out.excluded_ids_sha256=digest(excluded);return out;
}
}
