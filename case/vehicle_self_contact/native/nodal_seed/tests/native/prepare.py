"""Pinned original LAW42 contact-slot statements; no production value calls."""
from pathlib import Path
import argparse,hashlib,importlib.util,json
ROOT=Path(__file__).resolve().parent
def prepare(tl_root):
    helper=tl_root/'lib_utest/qualification/radioss_type25_selection/native/Sources.py'
    spec=importlib.util.spec_from_file_location('contact_slot_sources',helper)
    source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
    donors={}
    for row in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        data=(ROOT/row['path']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
        assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==row['git_blob_sha1']
        donors[Path(row['path']).name]=data.decode()
    def lines(name,first,last):return ''.join(donors[name].splitlines(keepends=True)[first-1:last])
    result={
      'reader_gs.inc':lines('hm_read_mat42.F',186,189),
      'reader_bulk.inc':lines('hm_read_mat42.F',197,198),
      'reader_slots.inc':lines('hm_read_mat42.F',252,254),
      'reader_pm100.inc':lines('hm_read_mat42.F',264,264),
      'generic_pm.inc':lines('hm_read_mat.F90',1464,1477),
      'reader_high.inc':lines('hm_read_mat.F90',1519,1521),
      'reader_keep_bulk.inc':lines('hm_read_mat.F90',1643,1646),
      'updated_high.inc':lines('updmat.F',422,423),
    }
    update=donors['law42_upd.F']
    begin='       IF (GAMA_INF <  ONE) THEN \n'
    after='       call my_alloc(STRETCH, NDATA, "STRETCH")\n'
    assert update.count(begin)==update.count(after)==1
    result['selected_update.inc']=begin+update.split(begin,1)[1].split(after,1)[0]
    # Preserve the full conditional block, including unchanged inactive writes
    # and formats. The remaining stability scan has no PM assignment.
    rest=update.split(after,1)[1]
    assert not __import__('re').search(r'\bPM\s*\(',rest,__import__('re').I)
    fmt=' 2000 FORMAT\n';end='c-----------\n      END'
    assert update.count(fmt)==update.count(end)==1
    result['selected_update_formats.inc']=fmt+update.split(fmt,1)[1].split(end,1)[0]
    gamma=[x for x in donors['updmat.F'].splitlines(keepends=True) if x.strip()=='GAMA_INF = ONE']
    assert gamma
    result['no_prony_gamma.inc']=gamma[0]
    assert 'pm(32,i) = bulk' in result['generic_pm.inc']
    assert 'if (ilaw /= 42)' in result['reader_keep_bulk.inc']
    assert 'PM(107,IMAT) = TWO*MAX(PM(32,IMAT),PM(100,IMAT))' in result['updated_high.inc']
    result['Constants.F90']=source.constants(donors['constant_mod.F'],list(result.values())).replace('selection_constants','law42_contact_slot_constants')
    result['ContactSlots.F']=(ROOT/'ContactSlots.F').read_text()
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--tl-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    for name,data in prepare(a.tl_root).items():
        path=a.output/name
        if a.check:assert path.read_text()==data
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(data)
    print('Original LAW42 reader/generic/update contact slots prepared')
