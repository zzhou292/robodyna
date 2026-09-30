          if(dt11==zero)dt11 = ep30
          do i=1,nel
            dx(i)=dx(i)/xl0(i)
            dxold(i)=dxold(i)/xl0(i)
            dpx(i)=dpx(i)/xl0(i)
            dpx2(i)=dpx2(i)/xl0(i)
            e(i)=e(i)/xl0(i)
          end do
!
          do i=1,nel
            fold(i)=fx(i)
            ddx(i)= (dx(i)-dxold(i))
            dvx(i)= ddx(i)/ dt11
            dvxs(i)= dvx(i)*ff(i)
          end do
