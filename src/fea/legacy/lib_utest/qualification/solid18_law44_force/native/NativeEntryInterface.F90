! SPDX-License-Identifier: AGPL-3.0-or-later
module REAR18_FORCE_ENTRY_INTERFACE
  use iso_c_binding, only:c_int,c_double
  implicit none
  interface
    subroutine REAR18_FORCE_CALCULATE(npts,curve,material,units,nodes,base,cursors, &
        global,saved,volume0,x,v,time,dt,now,nextcursor,nextglobal,nextsaved, &
        obs,forces,geometry,diagnostics,status)
      import c_int,c_double
      integer(c_int),intent(in) :: npts,units,nodes(8),cursors(8)
      real(c_double),intent(in) :: curve(2,npts+1),material(7),base(20,8),global(12)
      real(c_double),intent(in) :: saved(21),volume0,x(3,8),v(3,8),time,dt
      real(c_double),intent(out) :: now(20,8),nextglobal(12),nextsaved(21)
      real(c_double),intent(out) :: obs(38,8),forces(3,8),geometry(1111),diagnostics(8)
      integer(c_int),intent(out) :: nextcursor(8),status
    end subroutine
  end interface
end module
