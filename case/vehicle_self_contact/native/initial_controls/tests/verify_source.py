#!/usr/bin/env python3
from pathlib import Path
import argparse,json,hashlib
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);a=p.parse_args()
root=Path(__file__).parent;m=json.loads((root/'source-evidence.json').read_text());text={}
for row in m['files']:
 data=(a.workspace/row['path']).read_bytes()
 assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
 assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==row['git_blob']
 text[Path(row['path']).name]=data.decode()
c=text['convertcontacts.cxx'];rd=text['hm_read_inter_type25.F'];df=text['definter.F']
for value in ['double fric = lsdFD * lsdFSF','double radC5 = (lsdFS - lsdFD) * lsdFSF','double radC6 = -lsdDC',
 'interTypeVsMapDefaultVals["TYPE25"] = { {"Idel", 1}, {"Inacti", 5}, {"IGAP", 2} };']:
 assert value in c,value
for value in ['FLAGREMNOD = 2','IPARI(21)=1','IPARI(83) = IREM25I2','IF(IVIS2==0)IVIS2=1',
 'IF(STMAX==ZERO)STMAX=EP30','VISC=FIVEEM2','IF (MODFR==2.AND.IFQ<10) IFQ = IFQ + 10','IF (IFQ==10) XFILTR = ONE']:
 assert value in rd,value
for key in ['IREM25I2','ISHARP']:
 at=df.index("CASE ('"+key+"')",df.index('ELSEIF(ITYP == 25)'));assert 'DEF_DEF = 1' in df[at:at+110]
const=text['constant_mod.F']
for value in ['FIVEEM2   = ZEP05','ZEP05     = FIVE   / EP02','EP02  = HUNDRED','EP30  = EP20 * EP10']:
 assert value in const,value
assert 'STOPT=EP30' in rd.replace(' ','') and 'IF(STOPT==ZERO)STOPT=EP30' in rd.replace(' ','')
assert 'IDSENS=0' in rd.replace(' ','') and "HM_GET_INTV('ISENSOR',IDSENS" in rd.replace(' ','')
assert 'LSDYNA_DT                   = 1.0E+20;' in text['contact_automatic_single_surface.cfg']
assert '"Tstop", lsdDT' in c
assert 'BMUL0 = 0.20'  in text['machine.inc'] and 'PARAMETER(LVOXEL = 8000000)' in text['tri7box.F']
reader=text['hw_cfg_reader.cpp'];factory=text['mv_model_factory.cpp'];po=text['sdiModelViewPO.h'];manager=text['sdiIdManager.h']
assert 'int   a_id = 0;' in reader and 'a_title_str, a_id, a_unit_id)' in reader
assert 'if (!pre_object->GetId() && ss == "id")' in factory
assert 'SortState p_selectionSortMethod = SortById;' in po and 'if(0 == id) validId = GetNextAvailableId(type);' in po
assert 'p_preobjects[CFGType].push_back(pObj)' in po and 'p_pIdManager->AddId(validId, type)' in po
assert 'p_nextAvailableIdsOfIncludes[0].resize(p_mv.GetMaxEntityType() + 1, 1)' in manager
assert 'ReplaceNextAvailableId(id + 1, etype' in manager
interfaces=text['hm_read_interfaces.F']
assert "CALL HM_OPTION_START('/INTER')" in interfaces and 'OPTION_ID=NOINT' in interfaces and 'NI=NI+1' in interfaces
assert 'MECIReadModelBase::ReadKeyword' in text['meci_read_model_base.cpp']
assert 'a_cur_file_vect.push_back(a_read_file)' in text['meci_read_model_base.cpp'] and 'myIndexNewFile++' in text['meci_read_model_base.cpp']
# Exact complete converter corpus. No unlisted translation unit is silently
# presumed generator-neutral; the parent evidence pins all40 actual units.
tus=[row for row in m['files'] if '/dyna2rad/_private/' in row['path'] and row['path'].endswith('.cxx')]
assert len(tus)==40
node_creators=set()
for row in tus:
 t=(a.workspace/row['path']).read_text()
 if 'CreateNode(' in t:node_creators.add(Path(row['path']).name)
assert node_creators=={'convertnodes.cxx','convertsystems.cxx','convertelements.cxx','convertprops.cxx','convertrigids.cxx',
 'convertdefinetransform.cxx','convertcontrolvols.cxx','convertcrosssections.cxx','convertrwalls.cxx'}
simple=text['convertcontrolvols.cxx'].split('void sdiD2R::ConvertControlVolume::ConvertAirbagSimpleModel()',1)[1].split('void sdiD2R::ConvertControlVolume::ConvertAirbagAdiabaticGasModel()',1)[0]
assert 'CreateNode(' not in simple
assert 'ARCHINFO(18,1)=128' in text['archloops.inc'] and 'IBUILTIN=18' in text['machine.inc']
assert 'NVSIZ=ARCHINFO(IBUILTIN,1)' in text['contrl.F'].replace(' ','')
assert 'IPARI(I,NI) = 0' in interfaces
dispatch=text['hm_read_inter_struct.F']
assert "CASE ('TYPE25')" in dispatch and 'CALL HM_READ_INTER_TYPE25(' in dispatch
assert 'IPARI(39)' not in dispatch.replace(' ','') and 'IPARI(39)' not in rd.replace(' ','')
assert 'IPARI(39,NIN),INTBUF_TAB%IRECTM' in text['i25main_tri.F'].replace(' ','')
assert 'if(m_id == 0 )' in text['hm_rbodies_add_main_node.F90']
print(json.dumps({'status':'passed' ,'pinned_files':len(m['files']),'converter_units':len(tus),'population':'interval_not_exact_count'}))
