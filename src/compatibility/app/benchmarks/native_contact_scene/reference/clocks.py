"""Only scalar native clock/phase observation; never inspect inactive packet scratch."""
import gdb
from .gdb_access import Call, clock


class EarlyClocks(gdb.Breakpoint):
    def __init__(self, routine, names, emit, complete, count=3):
        super().__init__('*'+routine.lower()+'_',internal=True)
        self.routine,self.names,self.emit,self.complete=routine,names,emit,complete
        self.count=count
        self.seen=set()
        self.finished=False

    def stop(self):
        record=clock()
        key=(record['NCYCLE'],record['TT'],record['DT1'],record['DT12'])
        if key in self.seen:return False
        self.seen.add(key)
        call=Call(self.names)
        record.update(routine=self.routine,thread=gdb.selected_thread().num,
                      task=call.scalar('JTASK'))
        if 'JLT' in call.index:record['JLT']=call.scalar('JLT')
        self.emit('early_clock',record)
        if len(self.seen)>=self.count:
            self.finished=True
            self.enabled=False
        return self.complete()
