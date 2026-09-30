#include "RecurrenceReport.h"
#include <iostream>

int main(int argc,char** argv) {
  namespace io=crash::output; namespace audit=tl::qualification::qeph::recurrence;
  try {
    io::Require(argc==3,"Usage: qeph_recurrence_audit PROVENANCE.json NEW-DIRECTORY");
    const std::filesystem::path destination=argv[2];
    io::Require(!std::filesystem::exists(std::filesystem::symlink_status(destination)),"Audit directory must be new");
    const auto parent=destination.has_parent_path()?destination.parent_path():std::filesystem::path(".");
    io::Require(std::filesystem::is_directory(parent),"Audit parent directory must exist");
    const auto source=io::ReadBounded(argv[1],1024*1024); io::Document inventory;
    inventory.Parse(source.c_str(),source.size());
    io::Require(!inventory.HasParseError()&&inventory.IsObject()&&!inventory.ObjectEmpty(),"Provenance must be a nonempty JSON object");
    io::Document provenance; provenance.SetObject(); io::String(provenance,"inventory_sha256",io::Sha256(source));
    io::Integer(provenance,"inventory_bytes",source.size()); io::Value copy;
    copy.CopyFrom(inventory,provenance.GetAllocator()); provenance.AddMember("inventory",copy,provenance.GetAllocator());
    io::Require(std::filesystem::create_directory(destination),"Cannot create new audit directory");
    // External serialization is required by ArtifactIO. No claim of atomic
    // exclusive creation or a hermetic compiler/runtime is made.
    std::cout<<"Collecting frozen native full-state probes\n"<<std::flush;
    auto result=audit::CollectProbes();
    const auto raw=audit::EncodeReport(audit::DescribeAudit(result,provenance,true));
    io::WriteBytes(destination/"raw-matrices.json",raw);
    std::cout<<"Raw matrices retained before decision; analyzing feedback maps\n"<<std::flush;
    audit::Decide(result);
    const auto decision=audit::EncodeReport(audit::DescribeAudit(result,provenance,false,io::Sha256(raw),raw.size()));
    io::WriteBytes(destination/"decision.json",decision);
    std::cout<<"Native recurrence audit "<<(result.audit_passed?"passed":"rejected/unresolved")
             <<"; trajectory_execution_qualified=false\n";
    return result.audit_passed?0:2;
  } catch(const std::exception& e) {
    std::cerr<<"Recurrence report not completed: "<<e.what()<<'\n'; return 1;
  }
}
