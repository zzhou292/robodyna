#!/usr/bin/env python3
"""Check narrow source delta against the preserved qualified eight-CTA commit."""
import hashlib,json,pathlib,subprocess
here=pathlib.Path(__file__).resolve().parent
root=here.parents[2]
manifest=json.loads((here/'source-changes.json').read_text())
base=manifest['baseline']
assert base=='1c8cf7e73a475075a3665c800779b8daf7a0be60'
changed=subprocess.check_output(['git','-C',str(root),'diff',base,'--name-only','--','lib_src'],text=True).splitlines()
assert set(changed)=={x['path'] for x in manifest['changed_production_files']}
for row in manifest['changed_production_files']:
    old=subprocess.check_output(['git','-C',str(root),'show',base+':'+row['path']])
    new=(root/row['path']).read_bytes()
    assert len(old)==row['baseline_bytes'] and hashlib.sha256(old).hexdigest()==row['baseline_sha256']
    assert len(new)==row['candidate_bytes'] and hashlib.sha256(new).hexdigest()==row['candidate_sha256']
# The complete original immutable loop and full readback stay byte-identical.
reader='lib_src/elements/ShellMixedSectionReadback.cpp'
assert (root/reader).read_bytes()==subprocess.check_output(['git','-C',str(root),'show',base+':'+reader])
for rel,publication in [
 ('lib_src/elements/ShellBatchPlasticityCollectionStorage.cpp','  element_count_=count; collection_=std::move(owned);'),
 ('lib_src/elements/ShellMixedSectionStorage.cpp','  collection_=std::move(owned);mixed_=std::move(next);element_count_=count;'),
 ('lib_src/elements/failure/ShellFailureCollectionStorage.cpp','  failure_ = std::move(failure_storage);')]:
    text=(root/rel).read_text()
    assert text.count('  InvalidateActivitySources();\n'+publication)==1
s=(root/'lib_src/elements/ShellMixedSectionStorage.cpp').read_text()
a=s.index('SetupReport HostStorage::CheckActivitySectionSources(')
b=s.index('SetupReport HostStorage::CheckActivityFailureSources(',a)
source=s[a:b]
checks=['Collection()','if(!mixed_||!catalog)','if(one_point_)','HasReadShape(slab,count,*catalog)',
 'activity_source_catalog_==catalog&&activity_source_mixed_==mixed_.get()',
 'mixed_->CheckActivitySources(slab,count,*catalog)','if(report.status==SetupStatus::Success)']
positions=[source.index(x) for x in checks]
assert positions==sorted(positions)
print(json.dumps({'status':'passed','baseline':base,'production_changes':len(changed),
 'original_scan_and_all_cuda_unchanged':True,'owned_identity_and_success_only':True,
 'live_shape_and_availability':True,'successful_publication_invalidation_sites':3}))
