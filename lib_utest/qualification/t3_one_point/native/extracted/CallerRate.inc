      dt1   = dt1c(1) 
      dtinv = dt1 / max(dt1**2,em20)  ! inverse of dt
      asrate = one   ! to be changed for default value
#include "vectorize.inc"
      do i = 1,nel
        eps_k2 = (kxx(i)**2+kyy(i)**2+kxx(i)*kyy(i)+fourth*kxy(i)**2)
     .         * one_over_9*gbuf%thk(i)**2
        eps_m2 = four_over_3*(exx(i)**2+eyy(i)**2+exx(i)*eyy(i) + fourth*exy(i)**2)
        epsd_pg(i) = sqrt(eps_k2 + eps_m2)*dtinv
      end do
      gbuf%epsd(1:nel) = asrate * epsd_pg(1:nel) + (one - asrate) * gbuf%epsd(1:nel)      
!-------------------------------------------------------------------------------
