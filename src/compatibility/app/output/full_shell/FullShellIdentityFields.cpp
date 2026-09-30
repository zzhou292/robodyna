#include "FullShellIdentityFields.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::full_shell {
Document IdentityDocument(const Identity& i) {
    CheckIdentity(i);
    Document d;
    d.SetObject();
    Integer(d,"owner",i.owner);
    Integer(d,"run",i.run);
    Integer(d,"topology",i.topology);
    Integer(d,"source_instance",i.source_instance);
    Integer(d,"configuration",i.configuration);
    Integer(d,"qualification",i.qualification);
    Integer(d,"source_inventory_bytes",i.source_inventory_bytes);
    String(d,"source_inventory_sha256",i.source_inventory_sha256);
    String(d,"source_mapping_sha256",i.source_mapping_sha256);
    return d;
}
Identity ParseIdentity(const Value& v) {
    using namespace array_json;
    Keys(v,{"owner","run","topology","source_instance","configuration","qualification","source_inventory_bytes",
        "source_inventory_sha256","source_mapping_sha256"});
    Identity result{UInt(v["owner"]),UInt(v["run"]),UInt(v["topology"]),UInt(v["source_instance"]),UInt(v["configuration"]),
        UInt(v["qualification"]),UInt(v["source_inventory_bytes"]),Text(v["source_inventory_sha256"]),Text(v["source_mapping_sha256"])};
    CheckIdentity(result);
    return result;
}
} // namespace crash::output::full_shell
