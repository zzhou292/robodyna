"""Bounded reference-only single-thread accepted states and original force cohorts.

Runs GDB's own Engine child to natural completion. Debugger wall time is never
performance evidence. Main entry/return arrays are genuine source-defined state;
inactive FOR3 interpolation scratch is never read before its source assignment.
"""
import hashlib
import json
from pathlib import Path

import gdb
from .gdb_access import Call, clock, common, values

_abi=None
_stream=None
_failure=None
_exit_code=None
_records=0
_bytes=0
_main=None
_cycles=[]
_classification_packets=0
_packets=0
_geometry_packets=0
_responses=0
_pending=0
_breakpoints=[]


def emit(stage, record):
    global _records,_bytes
    text=json.dumps(dict(stage=stage,observation=record),allow_nan=False)+'\n'
    if _records>=20000 or _bytes+len(text.encode())>64<<20:
        raise ValueError('Native sequence observation resource cap exceeded')
    _stream.write(text);_stream.flush();_records+=1;_bytes+=len(text.encode())


def fail(error):
    global _failure
    _failure=str(error)
    return True


def extent(value,name,maximum=4096):
    if type(value) is not int or not 0<value<=maximum:
        raise ValueError('Native '+name+' outside bounded sequence scope')
    return value


def nodal(pointers, nodes, state=False):
    spec=[('A',3*nodes,'d'),('STIFN',nodes,'d')]
    if state:spec += [('X',3*nodes,'d'),('V',3*nodes,'d'),('MS',nodes,'d'),('ITAB',nodes,'i')]
    return {name:values(pointers[name],count,code) for name,count,code in spec}


class Returned(gdb.FinishBreakpoint):
    def __init__(self, routine, pointers, meta):
        global _pending
        super().__init__(gdb.newest_frame(),internal=True)
        self.routine,self.pointers,self.meta=routine,pointers,meta
        _pending+=1

    def stop(self):
        global _pending,_main
        try:
            data=dict(self.meta)
            if self.routine=='I25MAINF':
                data['arrays']=nodal(self.pointers,data['nodes'])
                _main=None
            elif self.routine=='I25DST3_3':
                data['arrays']={name:values(self.pointers[name],data['jlt'],'d')
                                for name in ('PENE','STIF')}
                data['history']={name:values(self.pointers[name],count*data['nsn'],code)
                                 for name,count,code in (('IRTLM',4,'i'),('PENE_OLD',5,'d'))}
            elif self.routine=='I25CDCOR3':
                data['arrays']={name:values(self.pointers[name],data['jlt'],'i')
                                for name in ('CAND_E_N','CAND_N_N')}
            else:
                data['arrays']={name:values(self.pointers[name],data['jlt'],'d')
                                for name in ('H1','H2','H3','H4','STIF')}
                h=data['arrays']
                active=[i for i in range(data['jlt']) if
                        ((h['H1'][i]+h['H2'][i])+h['H3'][i])+h['H4'][i]!=0]
                data['assembly_rows_one_based']=[i+1 for i in active]
                # N holds force after FOR3; zero-H lanes are not read by ASS0.
                data['active_force']={name:[values(self.pointers[name]+8*i,1,'d')[0] for i in active]
                                      for name in ('N1','N2','N3')}
                data['history']={name:values(self.pointers[name],count*data['nsn'],code)
                                 for name,count,code in (('IRTLM',4,'i'),('PENE_OLD',5,'d'),
                                                       ('STIF_OLD',2,'d'),('SECND_FR',6,'d'))}
            emit(self.routine.lower()+'_return',data)
            _pending-=1
            return False
        except Exception as error:return fail(error)

    def out_of_scope(self):fail('Sequence native call unwound before a defined return')


class Entered(gdb.Breakpoint):
    def __init__(self,routine):
        super().__init__('*'+routine.lower()+'_',internal=True)
        self.routine=routine

    def stop(self):
        global _main,_packets,_responses,_classification_packets,_geometry_packets
        try:
            if self.routine=='I25CDCOR3':
                caller=gdb.newest_frame().older()
                caller_name=caller.name() if caller else None
                # The same source helper packs classification inside COMP_2.
                # Only MAINF's later force cohorts belong to this observer.
                if caller_name=='i25comp_2_':
                    _classification_packets+=1
                    return False
                if caller_name!='i25mainf_':
                    raise ValueError('Unexpected CDCOR3 caller:'+str(caller_name))
            call=Call(_abi['routines'][self.routine]);now=clock()
            if 'JTASK' in call.index and call.scalar('JTASK')!=1:
                raise ValueError('This numerical sequence admits a single native worker only')
            meta={'clock':now,'NVSIZ':common('param_',1,'i')}
            if self.routine=='I25MAINF':
                if _main is not None:raise ValueError('Nested interface main is unsupported')
                nodes=extent(common('com04_',1,'i'),'nodes',128)
                if now['NSPMD']!=1 or call.scalar('NIN')!=1:raise ValueError('Single domain/interface required')
                if _cycles and now['NCYCLE']<=_cycles[-1]:raise ValueError('Native cycle did not advance')
                _cycles.append(now['NCYCLE']);meta['nodes']=nodes
                pointers={name:call.pointer(name) for name in ('X','V','MS','ITAB','A','STIFN')}
                meta['arrays']=nodal(pointers,nodes,True)
                _main=now['NCYCLE']
            else:
                if _main!=now['NCYCLE']:raise ValueError('Packet is outside its active main cycle')
                meta['jlt']=extent(call.scalar('JLT'),'JLT',4096)
                if meta['jlt']>meta['NVSIZ']:raise ValueError('Packet exceeds native NVSIZ')
                pointers={name:call.pointer(name) for name in call.index}
                if self.routine=='I25DST3_3':
                    _geometry_packets+=1
                    meta['nsn']=extent(call.scalar('NSN'),'secondary count',128)
                    meta['arrays']={name:call.array(name,meta['jlt'],'i') for name in ('CAND_N','CAND_E')}
                else:
                    meta['source_occurrences_one_based']=call.array('INDEX',meta['jlt'],'i')
                if self.routine=='I25CDCOR3':_packets+=1
                elif self.routine=='I25FOR3':
                    _responses+=1;meta['nsn']=extent(call.scalar('NSN'),'secondary count',128)
                    meta['arrays']={name:call.array(name,meta['jlt'],code) for name,code in
                                    (('CAND_N_N','i'),('PENE','d'),('STIF','d'))}
            emit(self.routine.lower()+'_entry',meta)
            # Return records retain scalar metadata and no entry arrays.
            returned={key:value for key,value in meta.items() if key!='arrays'}
            _breakpoints.append(Returned(self.routine,pointers,returned))
            return False
        except Exception as error:return fail(error)


def exited(event):
    global _exit_code
    _exit_code=getattr(event,'exit_code',None)


def install(path):
    global _abi,_stream
    if Path('native-sequence.jsonl').exists() or Path('native-sequence-summary.json').exists():
        raise FileExistsError('Preserve sequence outputs; use a fresh directory')
    _abi=json.loads(Path(path).read_text())
    if _abi['schema']!='robo_dyna.native_scene_observation_abi.v1':raise ValueError('Unknown sequence ABI')
    for pin in _abi['source_pins']:
        data=Path(pin['path']).read_bytes()
        if len(data)!=pin['bytes'] or hashlib.sha256(data).hexdigest()!=pin['sha256']:
            raise ValueError('Native ABI donor changed')
    _stream=Path('native-sequence.jsonl').open('x')
    gdb.events.exited.connect(exited)
    for name in ('I25MAINF','I25CDCOR3','I25DST3_3','I25FOR3'):_breakpoints.append(Entered(name))


def finish():
    complete=(_failure is None and _exit_code==0 and _pending==0 and _main is None and
              len(_cycles)>1 and _responses>0 and _geometry_packets>=_responses)
    record=dict(schema='robo_dyna.native_sequence_observation.v1',complete=complete,
                exit_code=_exit_code,failure=_failure,cycle_count=len(_cycles),
                first_cycle=_cycles[0] if _cycles else None,last_cycle=_cycles[-1] if _cycles else None,
                external_cdcor3_force_packets=_packets,geometry_packets=_geometry_packets,
                responses=_responses,records=_records,bytes=_bytes,
                excluded_classification_packets=_classification_packets,
                scope='Reference states/packet order only; debugger timing is not performance evidence')
    with Path('native-sequence-summary.json').open('x') as stream:json.dump(record,stream,indent=2);stream.write('\n')
    _stream.close()
    if not complete:
        if gdb.selected_inferior().pid:gdb.execute('kill')
        gdb.execute('quit 2')
