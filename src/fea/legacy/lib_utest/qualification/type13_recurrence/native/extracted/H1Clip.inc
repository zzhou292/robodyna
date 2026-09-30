          if(jecrou(1)>0)then
            do i=1,nel
              if(ifunc(i) /= 0.and.iecrou(i)==1)then
                if(fx(i)>=zero.and.fx(i)>yy(i))then
                  dpx(i)=dpx(i)+(fx(i)-yy(i))/xk(i)
                  fx(i)=yy(i)
                else if(fx(i)<zero.and.fx(i)<yy(i))then
                  dpx(i)=dpx(i)+(yy(i)-fx(i))/xk(i)
                  fx(i)=yy(i)
                end if
                fxep(i)=fx(i)
              end if
            end do
          end if
