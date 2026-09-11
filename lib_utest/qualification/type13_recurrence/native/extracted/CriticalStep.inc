        DO I=JFT,JLT
          IF(XCM(I)+XKM(I)<EM15)XM(I) =ONE
          XKM(I)= MAX(EM15,XKM(I))
          DT(I)=XM(I)/MAX(EM15,SQRT(XCM(I)*XCM(I)+XM(I)*XKM(I))+XCM(I))
          IF(XCR(I)+XKR(I)<EM15)XIN(I)=ONE
          XKR(I)= MAX(EM15,XKR(I))
          DTC(I)=XIN(I)/MAX(EM15,SQRT(XCR(I)*XCR(I)+XIN(I)*XKR(I))+XCR(I))
          DT(I)= MIN(DT(I),DTC(I))
        ENDDO
