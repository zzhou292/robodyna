         DO N=1,NUMNOD
          VDT = DT2*V(1,N)
          D(1,N)=D(1,N)+VDT
          X(1,N)=X(1,N)+VDT
          VDT = DT2*V(2,N)
          D(2,N)=D(2,N)+VDT
          X(2,N)=X(2,N)+VDT
          VDT = DT2*V(3,N)
          D(3,N)=D(3,N)+VDT
          X(3,N)=X(3,N)+VDT
        ENDDO
