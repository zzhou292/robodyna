        dtinv = dt1 / max(dt1**2,em20)  ! inverse of dt
#include "vectorize.inc"
        do i = 1,nel
          eps_k2 = (kxx(i)**2+kyy(i)**2+kxx(i)*kyy(i)+fourth*kxy(i)**2)
     .           * one_over_9*gbuf%thk(i)**2
          eps_m2 = four_over_3*(exx(i)**2+eyy(i)**2+exx(i)*eyy(i) + fourth*exy(i)**2)
          epsd_pg(i) = sqrt(eps_k2 + eps_m2)*dtinv
          epsd_glob(i) = epsd_glob(i) + epsd_pg(i) / npg
        end do
