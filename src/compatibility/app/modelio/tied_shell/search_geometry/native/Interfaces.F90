! SPDX-License-Identifier: MIT
module search_geometry_native_interfaces
  implicit none
  interface
    ! Exact argument order/ranks from the complete pinned INCOQ3 declaration.
    subroutine incoq3(irect,ixc,ixtg,nint,nel,neltg,is,geo,pm, &
        knod2elc,knod2eltg,nod2elc,nod2eltg,thk,nty,igeo,pm_stack,iworksh)
      use element_mod, only : nixc,nixtg
      use search_geometry_context, only : npropg,npropgi,npropm
      implicit none
      integer :: nint,nel,neltg,is,nty
      integer :: irect(4,*),ixc(nixc,*),ixtg(nixtg,*)
      integer :: knod2elc(*),knod2eltg(*),nod2elc(*),nod2eltg(*)
      integer :: igeo(npropgi,*),iworksh(3,*)
      double precision :: geo(npropg,*),pm(npropm,*),thk(*),pm_stack(20,*)
    end subroutine
    subroutine native_consumed_thickness(ixc,ixtg,ipartc,iparttg, &
        geo,thk,thk_part,nelc,neltg,consumed)
      use element_mod, only : nixc,nixtg
      use search_geometry_context, only : npropg
      implicit none
      integer :: ixc(nixc,*),ixtg(nixtg,*),ipartc(*),iparttg(*),nelc,neltg
      double precision :: geo(npropg,*),thk(*),thk_part(*),consumed(2)
    end subroutine
  end interface
end module
