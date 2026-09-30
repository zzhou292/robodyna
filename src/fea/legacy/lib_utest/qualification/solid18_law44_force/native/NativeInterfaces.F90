! SPDX-License-Identifier: AGPL-3.0-or-later
module REAR18_FORCE_INTERFACES
  use SOLID18_FORCE_PACKETS
  implicit none
  interface
    subroutine REAR18_FORCE_GEOMETRY(x,v,saved,nu,g,status)
      import native_geometry
      real(kind=8),intent(in) :: x(3,8),v(3,8),saved(21),nu
      type(native_geometry),intent(out) :: g
      integer,intent(out) :: status
    end subroutine
    subroutine REAR18_FORCE_POINT(g,ip,base,npts,curve,material,units,cursor, &
                                 time,dt,length,dsv,deg,now,obs,force,next,status)
      import native_geometry,MVSIZ
      type(native_geometry),intent(in) :: g
      integer,intent(in) :: ip,npts,units,cursor,deg
      real(kind=8),intent(in) :: base(20),curve(2,npts+1),material(7),time,dt,dsv
      real(kind=8),intent(inout) :: length,force(MVSIZ,8,3)
      real(kind=8),intent(out) :: now(20),obs(38)
      integer,intent(out) :: next,status
    end subroutine
    subroutine REAR18_FORCE_REDUCE(g,ip,now,obs,volume0,global,stiff,pp)
      import native_geometry
      type(native_geometry),intent(in) :: g
      integer,intent(in) :: ip
      real(kind=8),intent(in) :: now(20),obs(38),volume0
      real(kind=8),intent(inout) :: global(12),stiff,pp(1)
    end subroutine
    subroutine REAR18_FORCE_CONTROL(g,nodes,dsv,raw,deg)
      import native_geometry
      type(native_geometry),intent(in) :: g
      integer,intent(in) :: nodes(8)
      real(kind=8),intent(out) :: dsv(1)
      integer,intent(out) :: raw,deg
    end subroutine
    subroutine REAR18_FORCE_PRESSURE(g,pp,deg,force)
      import native_geometry,MVSIZ
      type(native_geometry),intent(in) :: g
      real(kind=8),intent(in) :: pp(1)
      integer,intent(in) :: deg
      real(kind=8),intent(inout) :: force(MVSIZ,8,3)
    end subroutine
  end interface
end module
