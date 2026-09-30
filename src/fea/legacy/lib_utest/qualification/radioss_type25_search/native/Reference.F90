! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete pinned I25XSAVE with explicit serial, converged, zero-curvature input.
subroutine rd_search_reference(nodes,ns,nm,no,secondary,main,one_d,x,saved) bind(c)
  use iso_c_binding
  use search_xsave_native, only: i25xsave
  implicit none
  integer(c_int),intent(in) :: nodes,ns,nm,no,secondary(*),main(*),one_d(*)
  real(c_double),intent(in) :: x(3,nodes)
  real(c_double),intent(inout) :: saved(3,min(nodes,ns+nm+no))
  real(c_double) :: xmin,ymin,zmin,xmax,ymax,zmax,cmax,curvature(1),sx,sy,sz,sx2,sy2,sz2
  integer :: active_main,connectivity(4,1)
  connectivity=1
  call i25xsave(x,secondary(1:ns),main(1:nm),ns,nm,0,saved, &
       xmin,ymin,zmin,xmax,ymax,zmax,cmax,curvature,0,connectivity,0, &
       sx,sy,sz,sx2,sy2,sz2,active_main,no,one_d(1:no),0,nodes,1,1)
end subroutine
