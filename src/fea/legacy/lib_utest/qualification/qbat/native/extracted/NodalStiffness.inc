         DO I=LFT,LLT
             A11 =PM(24,IMAT)
             VV = VISCMX * ALMIN(I)
             VV = VV*VV
             STI = HALF*THK(I) *AREA(I)* A11 / VV
             STIR = STI * THK(I)*THK(I) * ONE_OVER_12
             STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI
             STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI
             STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI
             STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI
             STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR
             STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR
             STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR
             STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR
             STRC(I) = STIR
         ENDDO
