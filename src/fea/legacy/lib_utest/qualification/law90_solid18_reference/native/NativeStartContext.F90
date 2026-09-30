! SPDX-License-Identifier: AGPL-3.0-or-later
! Test storage only; exact selected sizes are produced by native allocation tags.
module LAW90_START_MESSAGE_MOD
  implicit none
  integer, parameter :: MSGERROR=1, ANINFO=1
  integer :: geometry_errors=0
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1)
    integer, intent(in) :: MSGID,MSGTYPE,ANMODE,I1
    geometry_errors=geometry_errors+1
  end subroutine
end module
module LAW90_START_ELBUFDEF_MOD
  implicit none
  type G_BUFEL_
    integer :: G_SMSTR=0,G_JAC_I=0,G_ETOTSH=0
    real(kind=8), pointer :: SMSTR(:)=>null(),JAC_I(:)=>null()
  end type
  type L_BUFEL_
    real(kind=8), pointer :: JAC_I(:)=>null(),PIJ(:)=>null()
  end type
  type LAYER_
    integer :: L_JAC_I=0,L_PIJ=0,L_SIGL=0,L_VOL=0
    type(L_BUFEL_) :: LBUF(2,2,2)
  end type
  type ELBUF_STRUCT_
    type(G_BUFEL_) :: GBUF
    type(LAYER_) :: BUFLY(1)
  end type
contains
  subroutine MY_ALLOC(p,n,name)
    real(kind=8), pointer :: p(:)
    integer, intent(in) :: n
    character(*), intent(in) :: name
    allocate(p(n))
  end subroutine
end module
