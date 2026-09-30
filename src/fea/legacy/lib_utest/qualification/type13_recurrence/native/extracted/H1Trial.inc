          if(jecrou(1)+jecrou(3)>0)then
            do i=1,nel
              if(ifunc(i) /= 0.and.(iecrou(i)==1.or.iecrou(i)==3))then
                fx(i)=fxep(i)+xk(i)*ddx(i)
                if(fx(i)>=zero)then
                  xx(i)=dpx(i)+fx(i)/xk(i)
                else
                  xx(i)=-dpx(i)+fx(i)/xk(i)
                end if
              end if
            end do
          end if
