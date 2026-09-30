#!/usr/bin/env python3
"""Strict capture decoder: only defined fields, no numerical reconstruction."""
import argparse,hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def generated():
    raw=(ROOT/"native-structural.jsonl").read_bytes()
    manifest=json.loads((ROOT/"manifest.json").read_text())
    assert len(raw)==manifest["bytes"] and hashlib.sha256(raw).hexdigest()==manifest["sha256"]
    all_records=list(map(json.loads,raw.decode().splitlines()))
    stages={}
    for row in all_records:
        observation=row["observation"];key=(observation["clock"]["NCYCLE"],row["stage"])
        assert key not in stages;stages[key]=observation
        assert observation["MVSIZ"]==129 and observation["NIXC"]==7
        assert observation["impl1"]=={"IMPL_S":0,"IKPROJ":0}
        assert observation["clock"]["IRESP"]==0 and observation["clock"]["NSPMD"]==1
    names=["czcorc1_entry","czcorp5_entry","czcorp5_return","czprojn_entry","czprojn_return"]
    assert set(stages)=={(cycle,name) for cycle in (0,473,700) for name in names}
    text=['// Generated from the exact pinned packet; no production numerical calls.', '#include "Packet.h"',
      'namespace qeph_projection_test {', 'const std::array<CapturedRow,12>& CapturedRows() {',
      '  static const auto rows=[] {', '    std::array<CapturedRow,12> result{};']
    def emit(path,value):
        if isinstance(value,bool):literal="true" if value else "false"
        elif isinstance(value,int):literal=str(value)
        else:assert math.isfinite(value);literal=float(value).hex()
        text.append('    '+path+'='+literal+';')
    def matrix(path,values,columns):
        assert len(values)%columns==0
        for j in range(len(values)//columns):
            for k in range(columns):emit(path+f'[{j}][{k}]',values[j*columns+k])
    def get(stage,name,row,columns,warp=False):
        field=stage['warped_arrays' if warp else 'arrays'][name]
        ids=field['original_rows_zero_based'];values=field['compact_column_major']
        assert field['columns']==columns and len(values)==len(ids)*columns
        assert len(ids)==len(set(ids)) and all(0<=x<4 for x in ids)
        assert row in ids
        i=ids.index(row);return [values[i+len(ids)*k] for k in range(columns)]
    def geometry(path,stage,row,base=None):
        for name,source in [('area','AREA'),('area_i','AREA_I'),('x13','X13'),('x24','X24'),('y13','Y13'),('y24','Y24'),
                            ('mx13','MX13'),('my13','MY13'),('z1','Z1'),('ll','LL'),('l13','L13'),('l24','L24')]:
            owner=stage if source in stage['arrays'] else base
            emit(path+'.'+name,get(owner,source,row,1)[0])
        matrix(path+'.corel',get(stage,'COREL',row,8),2)
        vq=get(stage,'VQ',row,9)
        for i in range(3):
            for j in range(3):emit(path+f'.vq[{3*i+j}]',vq[i+3*j])
    def projection(path,stage,row,z):
        planar=bool(stage['planar'][row]);emit(path+'.planar',planar);emit(path+'.warped_defined',not planar);emit(path+'.z1',z)
        for name,columns in [('DI',6),('DB',12),('VQN',12)]:
            ids=stage['warped_arrays'][name]['original_rows_zero_based']
            assert ids==[i for i,x in enumerate(stage['planar']) if not x]
            if planar:continue
            values=get(stage,name,row,columns,True)
            if name=='DI':
                for k,v in enumerate(values):emit(path+f'.di[{k}]',v)
            else:matrix(path+'.'+name.lower(),values,3)
    for block,cycle in enumerate((0,473,700)):
        caller,rate,after,force,done=[stages[(cycle,name)] for name in names]
        assert caller['nodes']==18 and caller['elements']==4 and caller['JFT']==1 and caller['JLT']==4
        assert caller['controls']=={'NPT':3,'IDRIL':0,'ISMSTR':2,'NLAY':1,'IREP':0,'IXFEM':0}
        assert rate['controls']==after['controls']=={'NPT':3,'IDRIL':0,'TOL':1e-8}
        assert force['controls']==done['controls']=={'IDRIL':0,'IFINI':0}
        assert after['planar']==force['planar']==done['planar']
        for stage in (rate,after,force,done):
            for field in ['TT','DT1','DT12']:assert stage['clock'][field]==caller['clock'][field]
        assert len(caller['IXC'])==28
        for field in ['X','V','VR']:assert len(caller['arrays'][field])==54
        for row in range(4):
            path=f'result[{4*block+row}]'
            emit(path+'.cycle',cycle);emit(path+'.original_row',row)
            for field,key in [('time','TT'),('dt1','DT1'),('dt12','DT12')]:emit(path+'.'+field,caller['clock'][key])
            for field,key in [('ismstr','ISMSTR'),('nlay','NLAY'),('irep','IREP'),('ixfem','IXFEM')]:emit(path+'.'+field,caller['controls'][key])
            nodes=caller['IXC'][7*row+1:7*row+5]
            assert len(set(nodes))==4 and all(1<=n<=18 for n in nodes)
            for j,node in enumerate(nodes):
                emit(path+f'.native_node_indices[{j}]',node)
                for field,key in [('position','X'),('velocity','V'),('omega','VR')]:
                    for k in range(3):emit(path+f'.{field}[{j}][{k}]',caller['arrays'][key][3*(node-1)+k])
            for prefix in ['rate_entry','force_entry']:
                c=path+'.'+prefix+'.controls'
                for name,value in [('npt',3),('idril',0),('ifini',0),('iresp',0),('impl_s',0),('ikproj',0),('tolerance',1e-8)]:emit(c+'.'+name,value)
            entry=path+'.rate_entry';geometry(entry+'.geometry',rate,row)
            for name in ['V13','V24','VHI']:
                for k,v in enumerate(get(rate,name,row,3)):emit(entry+'.'+name.lower()+f'[{k}]',v)
            matrix(entry+'.rlxyz',get(rate,'RLXYZ',row,8),2)
            for j,node in enumerate(nodes):
                for k in range(3):emit(entry+f'.world_omega[{j}][{k}]',caller['arrays']['VR'][3*(node-1)+k])
            out=path+'.rate_expected'
            for name in ['V13','V24','VHI']:
                for k,v in enumerate(get(after,name,row,3)):emit(out+'.'+name.lower()+f'[{k}]',v)
            matrix(out+'.rlxyz',get(after,'RLXYZ',row,8),2)
            z=get({'arrays':{'Z1':after['Z1']}},'Z1',row,1)[0]
            projection(out+'.projection',after,row,z)
            entry=path+'.force_entry';geometry(entry+'.geometry',force,row,rate)
            projection(entry+'.projection',force,row,get(force,'Z1',row,1)[0])
            matrix(entry+'.vf',get(force,'VF',row,12),3);matrix(entry+'.vm',get(force,'VM',row,8),2)
            for j in range(4):
                for k in range(3):
                    emit(path+f'.force_expected.force[{j}][{k}]',get(done,f'F{k+1}{j+1}',row,1)[0])
                    emit(path+f'.force_expected.couple[{j}][{k}]',get(done,f'M{k+1}{j+1}',row,1)[0])
    text += ['    return result;','  }();','  return rows;','}','} // namespace qeph_projection_test']
    return '\n'.join(text)+'\n'
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();s=generated()
    if a.check:assert a.output.read_text()==s
    else:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(s)
    print('Pinned 12-row native projection packet fixture prepared')
