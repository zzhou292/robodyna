"""Bounded read-only native entries; invoked only inside GDB on its own child."""
import json
from pathlib import Path

import gdb

from .gdb_access import Call, clock, common, values

_abi = None
_records = {}
_failure = None
_stream = None
_breakpoints = []
_required = {'controls','main','inventory','classification','boundary','positive_response'}


def bounded(value, label, maximum=4096):
    if type(value) is not int or not 0 < value <= maximum:
        raise ValueError(f'Native {label} outside tiny-scene bounds:{value}')
    return value


def arrays(call, spec):
    return {name:call.array(name,count,code) for name,count,code in spec}


def controls(call):
    nodes=bounded(common('com04_',1,'i'),'node count')
    npari=bounded(common('param_',7,'i'),'NPARI',512)
    nin=bounded(call.scalar('NIN'),'interface index',64)
    raw=values(call.pointer('IPARI')+(nin-1)*npari*4,npari,'i')
    fields={'NRTM':4,'NSN':5,'NTY':7,'IVIS2':14,'source_interface_ID':15,'ILEV':20,
            'IGAP':21,'INACTI':22,'MFROT':30,'IFQ':31,'IGSTI':34,'INTTH':47,
            'IGAP0':53,'IEDGE':58,'FLAGREMNOD':63,'NADMSR':67,'NEDGE':68,'ISHARP':84}
    controls={name:raw[index-1] for name,index in fields.items()}
    for name in ('NRTM','NSN','NADMSR'):bounded(controls[name],name,16384)
    return dict(clock=clock(),nodes=nodes,npari=npari,nin=nin,controls=controls,ipari=raw)


def main(call):
    record=controls(call);nodes=record['nodes']
    record['arrays']=arrays(call,[('X',3*nodes,'d'),('V',3*nodes,'d'),('MS',nodes,'d'),
                                  ('ICODT',nodes,'i'),('ITAB',nodes,'i')])
    return record


def inventory(call):
    nr=bounded(call.scalar('NRTM'),'NRTM');ns=bounded(call.scalar('NSN'),'NSN')
    counts=call.array('KREMNOD',2*nr+1,'i')
    if any(v<0 for v in counts) or any(a>b for a,b in zip(counts,counts[1:])):
        raise ValueError('Unexpected native removal index layout')
    if counts[-1] > 1<<18:raise ValueError('Native removal extent exceeds probe cap')
    nodes=bounded(common('com04_',1,'i'),'node count')
    return dict(clock=clock(),
        controls={name:call.scalar(name) for name in ('NSN','NSNR','NRTM','NOINT','ILEV','FLAGREMNODE','IGAP')},
        scalars={name:call.scalar(name,'d') for name in ('MARGE','VMAXDT','BGAPSMX','PMAX_GAP','DRAD','DGAPLOAD')},
        removal_offsets=counts,removal_nodes=call.array('REMNOD',counts[-1],'i'),
        arrays=arrays(call,[('IRECT',4*nr,'i'),('NSV',ns,'i'),('STF',nr,'d'),('STFN',ns,'d'),
            ('GAP_S',ns,'d'),('GAP_M',nr,'d'),('CURV_MAX',nr,'d'),('MSEGTYP',nr,'i'),
            ('ICODT',nodes,'i'),('ISKEW',nodes,'i'),('XYZM',6,'d')]))


def classification(call):
    if call.scalar('JLT') == 0:return None
    nr=bounded(call.scalar('NRTM'),'NRTM');ns=bounded(call.scalar('NSN'),'NSN')
    jlt=bounded(call.scalar('JLT'),'JLT',4096)
    nadmsr=_records['controls']['controls']['NADMSR']
    return dict(clock=clock(),jlt=jlt,nrtm=nr,nsn=ns,nadmsr=nadmsr,
        arrays=arrays(call,[('IRECT',4*nr,'i'),('NSV',ns,'i'),('STF',nr,'d'),('STFN',ns,'d'),
            ('GAP_S',ns,'d'),('GAP_M',nr,'d'),('GAPN_M',4*nr,'d'),('MSEGTYP',nr,'i'),
            ('ADMSR',4*nr,'i'),('LBOUND',nadmsr,'i'),('NOD_NORMAL',12*nr,'f'),
            ('MVOISIN',4*nr,'i'),('CAND_N',jlt,'i'),('CAND_E',jlt,'i')]))


def boundary(call):
    if call.scalar('JLT') == 0:return None
    nadmsr=_records['controls']['controls']['NADMSR']
    nr=_records['controls']['controls']['NRTM']
    return dict(clock=clock(),
        scalars={name:call.scalar(name,'d') for name in ('MARGE','PENMIN','EPS0','DRAD','DGAPLOAD')},
        arrays=arrays(call,[('VTX_BISECTOR',6*nadmsr,'f'),('MSEGLO',nr,'i')]))


def response(call):
    if call.scalar('JLT') == 0:return None
    jlt=bounded(call.scalar('JLT'),'JLT',4096)
    penetration=call.array('PENE',jlt)
    selected=[i for i,v in enumerate(penetration) if v>0]
    if not selected:return None
    nr=bounded(call.scalar('NRTM'),'NRTM');ns=bounded(call.scalar('NSN'),'NSN')
    scalars={name:call.scalar(name,'d') for name in ('VISC','VISCF','ALPHA0','KMIN')}
    controls={name:call.scalar(name) for name in
              ('MFROT','IFQ','IORTHFRIC','INTTH','ILEV','INTEREFRIC','IGSTI','IVIS2','INACTI')}
    controls.update(KDTINT=common('scr18_',204,'i'),IDTMINS=common('sms_',6,'i'),
                    IDTMINS_INT=common('sms_',15,'i'),INCONV=common('impl1_',60,'i'))
    # Inactive lanes can contain unassigned interpolation scratch before FOR3
    # clears it. Read only source-defined positive-penetration lane operands.
    packet={}
    for names,code,width in ((('PENE','STIF','N1','N2','N3','H1','H2','H3','H4',
                               'FRICC','VISCFFRIC','MSI','VXI','VYI','VZI'),'d',8),
                            (('IX1','IX2','IX3','IX4','NSVG','CAND_N_N','CN_LOC','CE_LOC'),'i',4)):
        for name in names:
            packet[name]=[values(call.pointer(name)+i*width,1,code)[0] for i in selected]
    histories=[]
    for node in packet['CAND_N_N']:
        bounded(node,'local response history row',ns)
        row={'secondary_row_one_based':node}
        for name,count,code,width in (('PENE_OLD',5,'d',8),('STIF_OLD',2,'d',8),
                                      ('SECND_FR',6,'d',8),('IRTLM',4,'i',4)):
            row[name]=values(call.pointer(name)+(node-1)*count*width,count,code)
        histories.append(row)
    return dict(clock=clock(),jlt=jlt,selected_rows_one_based=[i+1 for i in selected],
                controls=controls,scalars=scalars,arrays=packet,histories=histories,
                main_segment_types=call.array('MSEGTYP',nr,'i'))


class Observe(gdb.Breakpoint):
    def __init__(self, routine, label, handler):
        super().__init__('*'+routine.lower()+'_',internal=True)
        self.routine,self.label,self.handler=routine,label,handler

    def stop(self):
        global _failure
        if self.label in _records:return False
        try:
            data=self.handler(Call(_abi['routines'][self.routine]))
            if data is not None:
                data['thread']=gdb.selected_thread().num
                _records[self.label]=data
                self.enabled=False
                print('Observed native scene stage '+self.label,flush=True)
                # Preserve completed bounded evidence even if a later stage fails.
                _stream.write(json.dumps({'stage':self.label,'observation':data},allow_nan=False)+'\n')
                _stream.flush()
        except Exception as error:
            _failure=str(error)
            return True
        return _required <= _records.keys()


def install(path):
    global _abi, _stream
    if Path('native-observations.jsonl').exists() or Path('native-observation-summary.json').exists():
        raise FileExistsError('Preserve prior native observations; use a fresh probe directory')
    _abi=json.loads(Path(path).read_text())
    if _abi['schema']!='robo_dyna.native_scene_observation_abi.v1':raise ValueError('Unknown native ABI')
    for item in _abi['source_pins']:
        import hashlib
        data=Path(item['path']).read_bytes()
        if len(data)!=item['bytes'] or hashlib.sha256(data).hexdigest()!=item['sha256']:
            raise ValueError('Native ABI donor changed')
    _stream=Path('native-observations.jsonl').open('x')
    for routine,label,handler in (('I25COMP_2','controls',controls),('I25MAINF','main',main),
            ('I25TRIVOX','inventory',inventory),
            ('I25COR3_22','classification',classification),('I25DST3_22','boundary',boundary),
            ('I25FOR3','positive_response',response)):
        _breakpoints.append(Observe(routine,label,handler))


def finish():
    complete=_failure is None and _required <= _records.keys()
    summary={'schema':'robo_dyna.native_scene_observation.v1','complete':complete,
             'scope':'read-only source entry observations; child intentionally stopped; no trajectory/performance acceptance',
             'stages':sorted(_records),'missing_stages':sorted(_required-_records.keys()),'failure':_failure}
    with Path('native-observation-summary.json').open('x') as out:
        json.dump(summary,out,indent=2,allow_nan=False);out.write('\n')
    _stream.close()
    if not complete:
        # The GDB process owns this one inferior; no existing workstation jobs.
        if gdb.selected_inferior().pid:gdb.execute('kill')
        gdb.execute('quit 2')
