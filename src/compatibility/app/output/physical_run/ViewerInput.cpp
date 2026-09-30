#include "ViewerInput.h"
#include "SourceInputFields.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/static_bundle/SourceAuthority.h"
namespace crash::output::physical_run {
namespace {
Document SourceFile(const records::RecordFile& f) {
    arrays::CheckRelativeName(f.file);
    arrays::CheckHash(f.sha256);
    Require(f.bytes>0 && f.bytes<=64u<<20,"Viewer source evidence exceeds cap");
    Document d;d.SetObject();
    String(d,"file",f.file);String(d,"sha256",f.sha256);Integer(d,"bytes",f.bytes);
    return d;
}
records::RecordFile ReadSourceFile(const Value& v) {
    using namespace array_json;
    Keys(v,{"file","sha256","bytes"});
    records::RecordFile f{Text(v["file"]),Text(v["sha256"]),static_cast<std::size_t>(UInt(v["bytes"]))};
    SourceFile(f);
    return f;
}
void Check(const ViewerInput& in) {
    arrays::CheckRelativeName(in.archive_directory);
    Require(in.archive_directory.find('/')==std::string::npos,
        "Viewer archive must be one named child directory");
    FileDocument(in.manifest);
    SourceFile(in.source.canonical_manifest);
    SourceFile(in.source.scope_report);
    SourceFile(in.source.source_member);
    records::source::CheckUnits(in.source.units);
    Require(!in.source.tire_policy.empty() && in.source.tire_policy.size()<=256,
        "Viewer source policy is missing or exceeds cap");
    arrays::CheckHash(in.mapping_sha256);
}
}
void AppendSourceInputs(Document& d,const records::source::SourceInputs& source) {
    records::source::CheckUnits(source.units);
    Require(!source.tire_policy.empty() && source.tire_policy.size()<=256,
        "Viewer source policy is missing or exceeds cap");
    array_json::Child(d,"canonical_manifest",SourceFile(source.canonical_manifest));
    array_json::Child(d,"scope_report",SourceFile(source.scope_report));
    array_json::Child(d,"source_member",SourceFile(source.source_member));
    array_json::Child(d,"source_authority",records::source::detail::AuthorityDocument(source));
}
records::source::SourceInputs ParseSourceInputs(const Value& d) {
    using namespace array_json;
    records::source::SourceInputs source;
    source.canonical_manifest=ReadSourceFile(d["canonical_manifest"]);
    source.scope_report=ReadSourceFile(d["scope_report"]);
    source.source_member=ReadSourceFile(d["source_member"]);
    const auto& a=d["source_authority"];
    Keys(a,{"canonical_manifest_sha256","canonical_manifest_bytes","scope_report_sha256",
        "scope_report_bytes","source_member_sha256","source_member_bytes","tire_policy",
        "source_mass_unit","source_length_unit","source_time_unit","mass_to_kg","length_to_m","time_to_s"});
    source.tire_policy=Text(a["tire_policy"]);
    source.units={Text(a["source_mass_unit"]),Text(a["source_length_unit"]),Text(a["source_time_unit"]),
        Real(a["mass_to_kg"]),Real(a["length_to_m"]),Real(a["time_to_s"])};
    records::source::detail::CheckAuthority(source,a);
    records::source::CheckUnits(source.units);
    Require(!source.tire_policy.empty() && source.tire_policy.size()<=256,
        "Viewer source policy is missing or exceeds cap");
    return source;
}
Document ViewerInputDocument(const ViewerInput& in) {
    Check(in);
    Document d;d.SetObject();
    String(d,"schema","robo_dyna.physical_viewer_input.v1");
    String(d,"archive_directory",in.archive_directory);
    array_json::Child(d,"manifest",FileDocument(in.manifest));
    AppendSourceInputs(d,in.source);
    String(d,"mapping_sha256",in.mapping_sha256);
    return d;
}
ViewerInput ParseViewerInput(const Value& d) {
    using namespace array_json;
    Keys(d,{"schema","archive_directory","manifest","canonical_manifest","scope_report",
        "source_member","source_authority","mapping_sha256"});
    Require(Text(d["schema"])=="robo_dyna.physical_viewer_input.v1","Unsupported viewer input receipt");
    ViewerInput in;
    in.archive_directory=Text(d["archive_directory"]);
    in.manifest=ReadFileRecord(d["manifest"]);
    in.source=ParseSourceInputs(d);
    in.mapping_sha256=Text(d["mapping_sha256"]);
    Check(in);
    return in;
}
records::RecordFile WriteViewerInput(const std::filesystem::path& root,const std::string& filename,
        const ViewerInput& in) {
    return WriteDocument(root,filename,ViewerInputDocument(in),ViewerInputByteCap);
}
ViewerInput ReadViewerInput(const std::filesystem::path& root,const records::RecordFile& expected) {
    return ParseViewerInput(array_json::Parse(ReadFile(root,expected,ViewerInputByteCap),ViewerInputByteCap));
}
std::filesystem::path ViewerArchivePath(const std::filesystem::path& root,const ViewerInput& in) {
    Check(in);
    Require(std::filesystem::symlink_status(root).type()==std::filesystem::file_type::directory,
        "Viewer receipt root is not a real directory");
    const auto path=std::filesystem::canonical(root)/in.archive_directory;
    Require(std::filesystem::is_directory(path) && !std::filesystem::is_symlink(path),
        "Viewer archive directory is missing or is a symbolic link");
    return path;
}
} // namespace crash::output::physical_run
