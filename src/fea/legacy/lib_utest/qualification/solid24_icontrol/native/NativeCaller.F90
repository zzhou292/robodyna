! SPDX-License-Identifier: AGPL-3.0-or-later
! One selected full controlled HEPH sequence, independently source-derived.
subroutine ic1_force_native(parameters,xref,x,v,jac_ref,volume0,base,time,values,stages,status) &
    bind(C,name='ic1_force_native')
  use iso_c_binding,only:c_double,c_int,c_int64_t
  use ieee_arithmetic,only:ieee_is_finite
  use HEPH_NATIVE_PACKETS
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  real(c_double),intent(in)::parameters(4),xref(3,8),x(3,8),v(3,8),jac_ref(10),volume0,base(22),time(2)
  real(c_double),intent(inout)::values(93)
  integer(c_int64_t),intent(out)::stages
  integer(c_int),intent(out)::status
  interface
    subroutine material(p,b,mf,d,step,out,code) bind(C,name='law42_solid_caller_native')
      import c_double,c_int
      real(c_double),intent(in)::p(4),b(9),mf(9),d(6),step(4)
      real(c_double),intent(out)::out(33)
      integer(c_int),intent(out)::code
    end subroutine
    subroutine slots_native(p,s) bind(C,name='ic1_native_slots')
      import c_double
      real(c_double),intent(in)::p(4)
      real(c_double),intent(out)::s(6)
    end subroutine
    subroutine HEPH_FORCE_GEOMETRY(xref,x,v,jac_ref,dt,g,status)
      import c_double,geometry_packet
      real(c_double),intent(in)::xref(3,8),x(3,8),v(3,8),jac_ref(10),dt
      type(geometry_packet),intent(out)::g
      integer,intent(out)::status
    end subroutine
  end interface
  type(geometry_packet)::geometry
  real(c_double)::result(93),pm(110,1),slots(6),m(33),step(4),old_hour(3,4),hour(1,3,4)
  real(c_double)::energy(MVSIZ),force(MVSIZ,8,3),sti(MVSIZ),distor(1),assembled(3,8),nodal(8)
  real(c_double)::hg_sti,dist_sti
  integer::k,n,h,cursor
  stages=0;status=1;center_contacts=0;corner_contacts=0
  distortion_sigma=zero;distortion_parameters=zero;distortion_flag=0
  if(.not.all(ieee_is_finite(parameters)).or..not.all(ieee_is_finite(xref)).or. &
     .not.all(ieee_is_finite(x)).or..not.all(ieee_is_finite(v)).or. &
     .not.all(ieee_is_finite(jac_ref)).or..not.all(ieee_is_finite(base)).or. &
     .not.all(ieee_is_finite(time)).or..not.ieee_is_finite(volume0))return
  if(parameters(1)<=0.or.parameters(3)<=0.or.parameters(4)<=0.or.volume0<=0.or. &
     parameters(2)<0.or.parameters(2)>0.48999.or.time(2)<0)return
  ! The higher-nu controlled SDLEN8 path is not silently substituted by SDLEN3.
  if(.not.ieee_is_finite(time(1)+time(2)))return
  if(time(2)>0.and.time(1)+time(2)<=time(1))return
  call HEPH_FORCE_GEOMETRY(xref,x,v,jac_ref,time(2),geometry,status)
  if(status/=0)return
  stages=1
  step=[time(2),geometry%volume(1),volume0,geometry%length(1)]
  call material(parameters,base(1:9),geometry%gradient(1,:),geometry%rate(1,:),step,m,status)
  if(status/=0)return
  if(.not.all(ieee_is_finite(m)))then
    status=3;return
  endif
  stages=3
  call slots_native(parameters,slots)
  pm=zero;pm(1,1)=parameters(3)
  pm(20,1)=slots(1);pm(21,1)=slots(2);pm(22,1)=slots(3)
  pm(32,1)=slots(4);pm(100,1)=slots(5);pm(107,1)=slots(6)
  do k=1,3
    do h=1,4
      old_hour(k,h)=base(9+4*(k-1)+h)
    enddo
  enddo
  hourglass_work=zero;hourglass_calls=0
  call IC1_HOURGLASS(pm,m,geometry,time(2),volume0,old_hour,hour,energy,force,sti)
  if(hourglass_calls/=1)then
    status=4;return
  endif
  hg_sti=sti(1);stages=7
  call IC1_MATERIAL_FORCES(m,geometry,force)
  stages=31
  distor=base(22)
  call IC1_DISTORTION(pm,m,geometry,x,v,time(2),distor,force,sti)
  dist_sti=sti(1);stages=127
  call IC1_ASSEMBLE(force,sti,assembled,nodal)
  stages=255
  result(1:9)=m(1:9);result(8)=energy(1)
  do k=1,3
    do h=1,4
      result(9+4*(k-1)+h)=hour(1,k,h)
    enddo
  enddo
  result(22)=distor(1)
  cursor=23
  do n=1,8
    result(cursor:cursor+2)=assembled(:,n);cursor=cursor+3
  enddo
  result(47:79)=m
  result(80:82)=[m(33),hg_sti,dist_sti]
  result(83:90)=nodal
  ! Expose actual energy-density before/after and cumulative distortion values.
  ! These increments are observations, not reused as the carried/native updates.
  result(91)=hourglass_work
  result(92)=distor(1)-base(22)
  result(93)=time(1)+time(2)
  if(.not.all(ieee_is_finite(result)).or.any(nodal<=0))then
    status=3;return
  endif
  values=result;status=0
end subroutine
