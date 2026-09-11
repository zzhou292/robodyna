! SPDX-License-Identifier: AGPL-3.0-or-later
! Observed full family recurrence. Inputs use native source-slot order.
subroutine heph_force_native(parameters,xref,x,v,jac_ref,volume0,base,time,values,status) &
    bind(C,name='heph_force_native')
  use iso_c_binding,only:c_double,c_int
  use HEPH_NATIVE_PACKETS
  implicit none
  real(c_double),intent(in) :: parameters(4),xref(3,8),x(3,8),v(3,8),jac_ref(10),volume0
  real(c_double),intent(in) :: base(21),time(2)
  real(c_double),intent(out) :: values(187)
  integer(c_int),intent(out) :: status
  interface
    subroutine material(p,b,mf,d,step,out,code) bind(C,name='law42_solid_caller_native')
      import c_double,c_int
      real(c_double),intent(in) :: p(4),b(9),mf(9),d(6),step(4)
      real(c_double),intent(out) :: out(33)
      integer(c_int),intent(out) :: code
    end subroutine
    subroutine HEPH_FORCE_GEOMETRY(xref,x,v,jac_ref,dt,g,status)
      import c_double,geometry_packet
      real(c_double),intent(in) :: xref(3,8),x(3,8),v(3,8),jac_ref(10),dt
      type(geometry_packet),intent(out) :: g
      integer,intent(out) :: status
    end subroutine
    subroutine HEPH_FORCE_REDUCE(p,v0,old_hour,old_stress,m,g,hour,energy,f,hg)
      import c_double,geometry_packet,MVSIZ
      real(c_double),intent(in) :: p(4),v0,old_hour(3,4),old_stress(6),m(33)
      type(geometry_packet),intent(in) :: g
      real(c_double),intent(out) :: hour(1,3,4),energy(1),f(MVSIZ,8,3),hg(3)
    end subroutine
  end interface
  type(geometry_packet) :: geometry
  real(c_double) :: m(33),step(4),old_hour(3,4),hour(1,3,4),energy(1),force(MVSIZ,8,3),hg(3)
  integer :: k,n,h,cursor
  values=zero
  status=0
  call HEPH_FORCE_GEOMETRY(xref,x,v,jac_ref,time(2),geometry,status)
  if(status/=0)return
  step=[time(2),geometry%volume(1),volume0,geometry%length(1)]
  call material(parameters,base(1:9),geometry%gradient(1,:),geometry%rate(1,:),step,m,status)
  if(status/=0)return
  do k=1,3
    do h=1,4
      old_hour(k,h)=base(9+4*(k-1)+h)
    enddo
  enddo
  call HEPH_FORCE_REDUCE(parameters,volume0,old_hour,base(1:6),m,geometry,hour,energy,force,hg)
  values(1:9)=m(1:9)
  values(8)=energy(1)
  do k=1,3
    do h=1,4
      values(9+4*(k-1)+h)=hour(1,k,h)
    enddo
  enddo
  values(22)=time(1)+time(2)
  cursor=23
  do n=1,8
    call append(force(1,n,:))
  enddo
  call append(geometry%frame(1,:))
  do n=1,8
    call append(geometry%x(1,n,:))
  enddo
  do n=1,8
    call append(geometry%v(1,n,:))
  enddo
  call append([geometry%volume(1),geometry%length(1)])
  do k=1,3
    call append(geometry%p(1,:,k))
  enddo
  do n=1,4
    call append(geometry%ph(1,n,:))
  enddo
  call append([geometry%jac(1,1),geometry%jac(1,5),geometry%jac(1,7)])
  call append(geometry%gradient(1,:))
  call append(geometry%rate(1,:))
  call append(m)
  call append(hg)
  if(cursor/=188)error stop 'HEPH observation packet extent mismatch'
contains
  subroutine append(packet)
    real(c_double),intent(in) :: packet(:)
    if(cursor+size(packet)>188)error stop 'HEPH observation packet overflow'
    values(cursor:cursor+size(packet)-1)=packet
    cursor=cursor+size(packet)
  end subroutine
end subroutine
