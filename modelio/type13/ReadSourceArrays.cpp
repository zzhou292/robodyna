#include "ReadInternal.h"
namespace crash::modelio::type13::reader {
void ReadSourceArrays(const Value& arrays) {
    struct Expected { const char* name;const char* dtype;const char* hash;std::size_t bytes,rows,columns;const char* fields; };
    // Exact existing canonical array declarations; no data is reconstructed.
    constexpr Expected expected[]={
    {"node_ids","<u8","14e5f078598e832836317394913e276a066cac121cf652335762168bb2b0d940",3145320,393165,1,"source_node_id"},
    {"node_positions","<f8","b15188d013bcae29435a583c42adb2ecf5941369fb72c2304b5e986674a164f1",9435960,393165,3,"x_m,y_m,z_m"},
    {"node_source_lines","<u4","fdb91194914d3c609bb285ce6c2bb136d49d818f6088f5067677c618a5f621c1",1572660,393165,1,"source_line"},
    {"node_blank_masks","<u2","883dd1d9da7eb174f5d13f51ce18958fcfa9b61e9778637709647498ebdf8dfa",786330,393165,1,"blank_field_mask"},
    {"node_codes","<i4","4e799f5e8856cab88a731199e51687a609ba335430ceb20bdd2a29ad2ff193f1",3145320,393165,2,"tc_raw,rc_raw"},
    {"beams_records","<u8","5e83464fd42261a9e8438e224c5255870431fc5c7959c62a6c948cb1533c7041",374800,4685,10,"element_id,part_id,n1,n2,n3,rt1,rr1,rt2,rr2,local"},
    {"beams_source_lines","<u4","fa3d8bcdec9eaa8395147f58584c4fc3742d1d8e1659e503b8e94ff54cc84d42",18740,4685,1,"source_line"},
    {"beams_blank_masks","<u2","80eafbad479d216b9c8f31b9c67e6ea6c1c6855e13b10b323cad21dd0add3a8b",9370,4685,1,"blank_field_mask"}
    };
    Require(arrays.IsObject()&&arrays.MemberCount()==8,"TYPE13 canonical array inventory changed");
    for(const auto& e:expected) {
        const auto& value=Member(arrays,e.name);Require(value.IsObject()&&value.MemberCount()==6,"TYPE13 canonical array declaration shape changed");
        TextIs(value,"dtype",e.dtype);TextIs(value,"sha256",e.hash);
        Require(Text(value,"file")==std::string("arrays/")+e.name+".bin"&&Unsigned(value,"bytes")==e.bytes,
            "TYPE13 canonical array path/byte count changed");
        const auto& shape=Array(value,"shape",2,2);
        Require(Unsigned(shape[0])==e.rows&&Unsigned(shape[1])==e.columns,"TYPE13 canonical array shape changed");
        const auto& fields=Array(value,"fields",e.columns,e.columns);std::string joined;
        for(const auto& field:fields.GetArray()){if(!joined.empty())joined+=",";joined+=Text(field);}
        Require(joined==e.fields,"TYPE13 canonical array field identity changed");
    }
}
}
