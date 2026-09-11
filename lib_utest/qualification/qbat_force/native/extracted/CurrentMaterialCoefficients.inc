           MX  =MAT(JFT)
           DO I=JFT,JLT
             THK02(I) = THK0(I)*THK0(I)
             VOLG(I) = THK0(I)*AREA(I)
             DT1C(I) = DT1
             RHO(I)=PM(1,MX)
             IPID=PID(I)
             YM(I) =PM(20,MX)
             NU(I)  =PM(21,MX)
             G(I)  =PM(22,MX)
             A11(I) =PM(24,MX)
             A12(I) =PM(25,MX)
             SSP(I) =PM(27,MX)
             KRZ(I) =KFAC*G(I)
           ENDDO
