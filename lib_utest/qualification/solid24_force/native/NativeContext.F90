! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only records and dormant context; no geometry or material expressions.
module HEPH_NATIVE_PACKETS
  use HEPH_NATIVE_CONSTANT_MOD
  implicit none
#include "mvsiz_p.inc"
  type geometry_packet
    real(kind=8) :: x(MVSIZ,8,3),v(MVSIZ,8,3),frame(MVSIZ,9)
    real(kind=8) :: p(MVSIZ,4,3),ph(MVSIZ,4,4),jac(MVSIZ,7)
    real(kind=8) :: volume(MVSIZ),length(MVSIZ),gradient(MVSIZ,9),rate(MVSIZ,6)
  end type
end module

module HEPH_NATIVE_HOUR_OBSERVATIONS
  implicit none
  real(kind=8) :: work(2)=0,modulus=0,viscosity=0
end module
subroutine HEPH_NATIVE_HOUR_OBSERVE(half_step,index,value,gg,fcl)
  use HEPH_NATIVE_HOUR_OBSERVATIONS
  implicit none
  integer,intent(in) :: half_step,index
  real(kind=8),intent(in) :: value,gg,fcl
  if(index/=1.or.half_step<1.or.half_step>2) error stop 'Bad hourglass observation'
  work(half_step)=value
  modulus=gg
  viscosity=fcl
end subroutine

module HEPH_NATIVE_MESSAGE_MOD
  implicit none
  integer,parameter :: ANINFO=1
  integer :: geometry_errors=0
contains
  subroutine ANCMSG(MSGID,ANMODE,I1)
    integer,intent(in) :: MSGID,ANMODE,I1
    geometry_errors=geometry_errors+1
  end subroutine
end module

module HEPH_NATIVE_ALE_MOD
  implicit none
  type global_context
    integer :: ICAA=0,ISFINT=0
  end type
  type ale_context
    type(global_context) :: GLOBAL
  end type
  type(ale_context) :: ALE
end module

module HEPH_NATIVE_ALEFVM_MOD
  implicit none
  type context
    integer :: IEnabled=0
  end type
  type(context) :: ALEFVM_Param
end module

module HEPH_NATIVE_ALEANIM_MOD
  implicit none
  type FANI_CELL_
    logical :: IS_VORT_X_REQUESTED=.false.,IS_VORT_Y_REQUESTED=.false.,IS_VORT_Z_REQUESTED=.false.
    real(kind=8) :: VORT_X(1),VORT_Y(1),VORT_Z(1)
  end type
end module

module HEPH_NATIVE_ELBUFDEF_MOD
  implicit none
  type global_buffer
    integer :: G_PLA=0
  end type
  type point_buffer
    real(kind=8) :: SEQ(1)=0
  end type
  type layer_buffer
    integer :: L_SEQ=0
    type(point_buffer) :: LBUF(1,1,1)
  end type
  type ELBUF_STRUCT_
    type(global_buffer) :: GBUF
    type(layer_buffer) :: BUFLY(1)
  end type
end module

module HEPH_NATIVE_MATPARAM_DEF_MOD
  implicit none
  type matparam_struct_
    integer :: crit_plas=0
  end type
end module

subroutine HEPH_NATIVE_ARRET(code)
  integer,intent(in) :: code
  error stop 'HEPH native geometry left selected positive-Jacobian scope'
end subroutine
subroutine HEPH_NATIVE_SLENA()
  error stop 'HEPH native caller entered unsupported ALE length'
end subroutine
subroutine HEPH_NATIVE_SLDEGE()
  error stop 'HEPH native caller entered unsupported DTSDE'
end subroutine
subroutine HEPH_NATIVE_SZSVM()
  error stop 'HEPH native caller entered unsupported plastic hourglass cap'
end subroutine
subroutine HEPH_NATIVE_MDAMA24()
  error stop 'HEPH native caller entered unsupported damage law'
end subroutine
subroutine HEPH_NATIVE_SZSTRAINHG()
  error stop 'HEPH native caller entered unrequested strain observation'
end subroutine
