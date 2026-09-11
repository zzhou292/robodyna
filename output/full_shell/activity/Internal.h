#pragma once
#include "ActivityRecord.h"

namespace crash::output::full_shell::activity::detail {
std::size_t WordCount(std::size_t);
std::size_t Preflight(const Context&,Limits);
arrays::Layout Layout(std::size_t);
void CheckWords(const Context&,const std::vector<std::uint64_t>&);
Document Declaration(const Context&);
void CheckDeclaration(const Context&,const Value&);
Document Frame(const ActivityRecord&,const arrays::Descriptor&);
arrays::Descriptor ParseFrame(const Context&,const Value&,const FrameStamp&);
std::string EncodeDocument(const Document&);
Document ReadDocument(const std::filesystem::path&,const RecordFile&);
} // namespace crash::output::full_shell::activity::detail
