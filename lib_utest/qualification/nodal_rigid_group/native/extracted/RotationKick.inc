        DO N=1,NUMNOD
          VR(1,N)=VR(1,N)+DT12*AR(1,N)
          VR(2,N)=VR(2,N)+DT12*AR(2,N)
          VR(3,N)=VR(3,N)+DT12*AR(3,N)
          AR(1,N)=ZERO
          AR(2,N)=ZERO
          AR(3,N)=ZERO      
        ENDDO
