      module kinchk_interfaces
      implicit none
      interface
      subroutine kinchk(ikine,rwl,itab,nprw,lprw,kinet,npby,lpby,irbe2,lrbe2,irbe3,lrbe3,
     .                 nom_opt,ptr_nopt_rwall,ptr_nopt_rbe2,ptr_nopt_rbe3,itagcyc)
#include "com04_c.inc"
      integer ikine(*),itab(*),nprw(*),lprw(*),kinet(*),npby(nnpby,*),lpby(*)
      integer irbe2(nrbe2l,*),lrbe2(*),irbe3(nrbe3l,*),lrbe3(*),nom_opt(lnopt1,*)
      integer ptr_nopt_rwall,ptr_nopt_rbe2,ptr_nopt_rbe3,itagcyc(*)
      real(kind=8) rwl(nrwlp,*)
      end subroutine
      subroutine kinini(ikine)
      integer ikine(*)
      end subroutine
      end interface
      end module
