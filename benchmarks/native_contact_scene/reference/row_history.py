"""Reference-only same-cycle ownership of complete persistent native row arrays."""
import gdb
from .gdb_access import Call, clock, values

_FIELDS=(('IRTLM',4,'i'),('PENE_OLD',5,'d'),('STIF_OLD',2,'d'),
         ('SECND_FR',6,'d'),('TIME_S',1,'d'),('ICONT_I',1,'i'))


class Rows(gdb.Breakpoint):
    def __init__(self,names,emit,fail):
        super().__init__('*i25optcd_',internal=True)
        self.names,self.emit,self.fail=names,emit,fail
        self.cycle=None;self.count=0;self.pointers={};self.snapshots=0;self.initial=False

    def stop(self):
        try:
            call=Call(self.names);now=clock();count=call.scalar('NSN')
            if not 0<count<=128 or call.scalar('ITASK')!=0:
                raise ValueError('Row history observer requires bounded single-worker scope')
            if self.count and count!=self.count:raise ValueError('Native persistent row extent changed')
            self.count=count;self.cycle=now['NCYCLE']
            self.pointers={name:call.pointer(name) for name,_,_ in _FIELDS}
            if not self.initial:
                if self.cycle!=0:raise ValueError('Initial native ICONT_I was not observed at cycle0')
                self.emit('initial_rows_after_begin',dict(clock=now,rows=self.read(self.cycle)))
                self.initial=True
            return False
        except Exception as error:return self.fail(error)

    def read(self,cycle):
        # The source rebinds these persistent arrays in OPTCD before every
        # MAINF, after IRTLM rollover. Do not reuse addresses across a skipped
        # cycle or claim a stale snapshot if source scheduling changes.
        if self.cycle!=cycle or not self.pointers:
            raise ValueError('Native history pointers are not bound in this main cycle')
        return {name:values(self.pointers[name],width*self.count,code) for name,width,code in _FIELDS}

    def after_main(self,cycle):
        result=self.read(cycle);self.snapshots+=1;return result


class Coefficients(gdb.Breakpoint):
    def __init__(self,names,emit,fail):
        super().__init__('*i25cor3_3_',internal=True)
        self.names,self.emit,self.fail=names,emit,fail
        self.observed=None;self.calls=0

    def stop(self):
        try:
            call=Call(self.names)
            current={name:call.scalar(name,'d') for name in ('KMIN','KMAX')}
            current.update(IGSTI=call.scalar('IGSTI'),ISTIF_MSDT=call.scalar('ISTIF_MSDT'))
            if self.observed is None:
                self.observed=current
                self.emit('coefficient_controls',dict(clock=clock(),controls=current))
            elif current!=self.observed:raise ValueError('Native coefficient controls changed within fixed source scene')
            self.calls+=1
            return False
        except Exception as error:return self.fail(error)
