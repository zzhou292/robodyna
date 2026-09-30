            DO J=1,3
             X(J,M)=X(J,M)*MASRB+X(J,M_I)*RBY(14,I)
            ENDDO
            MASRB = MASRB + RBY(14,I)
            DO J=1,3
              X(J,M)=X(J,M)/MASRB
            ENDDO
