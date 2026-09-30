        DO L=1,NRTM
          ! CONNECTIVITES ELEMENT
          N1=IRECT(1,L)
          N2=IRECT(2,L)
          N3=IRECT(3,L)
          N4=IRECT(4,L)
          ! LONGUEUR DIAG 1
          DX1=(X(1,N1)-X(1,N3))
          DY1=(X(2,N1)-X(2,N3))
          DZ1=(X(3,N1)-X(3,N3))
          segment_data(l,1) = max(segment_data(l,1),sqrt(dx1**2+dy1**2+dz1**2))
          DD=MAX(DD,SQRT(DX1**2+DY1**2+DZ1**2))
          ! LONGUEUR DIAG 2
          DX3=(X(1,N2)-X(1,N4))
          DY3=(X(2,N2)-X(2,N4))
          DZ3=(X(3,N2)-X(3,N4))
          segment_data(l,1) = max(segment_data(l,1),sqrt(dx3**2+dy3**2+dz3**2))
          DD=MAX(DD,SQRT(DX1**2+DY1**2+DZ1**2))
        ENDDO
