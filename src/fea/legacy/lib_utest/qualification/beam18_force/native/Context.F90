! SPDX-License-Identifier: AGPL-3.0-or-later
! Private NEL1/NPT4 original selected buffer context, not a native solver ABI.
module B18F_MATPARAM_DEF_MOD
  type MATPARAM_STRUCT_
    integer :: FAIL(1)=0
  end type
  type TTABLE
    integer :: unused=0
  end type
end module
module B18F_MAT_ELEM_MOD
  use B18F_MATPARAM_DEF_MOD
end module
module B18F_ELBUFDEF_MOD
  type L_BUFEL_
    real(8),pointer :: SIG(:)=>null(),STRA(:)=>null(),PLA(:)=>null()
    real(8),pointer :: DMGSCL(:)=>null(),FRAC(:)=>null(),SIGB(:)=>null(),SEQ(:)=>null()
  end type
  type MATBUF_
    real(8),pointer :: VAR(:)=>null()
    integer,pointer :: VARTMP(:)=>null()
  end type
  type BUF_LAY_
    integer :: L_PLA=1,L_STRA=3,L_DMGSCL=0,L_SIGB=0,NVAR_MAT=1,NVARTMP=1
    type(L_BUFEL_),pointer :: LBUF(:,:,:)=>null()
    type(MATBUF_),pointer :: MAT(:,:,:)=>null()
  end type
  type G_BUFEL_
    integer :: G_WPLA=1
    real(8),pointer :: WPLA(:)=>null(),MAXFRAC(:)=>null(),MAXEPS(:)=>null()
  end type
  type ELBUF_STRUCT_
    type(BUF_LAY_),pointer :: BUFLY(:)=>null()
    type(G_BUFEL_),pointer :: GBUF=>null()
  end type
end module
module B18F_MY_ALLOC_MOD
contains
  subroutine B18F_MY_ALLOC(value,n,m,name)
    real(8),allocatable,intent(inout) :: value(:,:)
    integer,intent(in) :: n,m
    character(*),intent(in) :: name
    if(n/=1.or.m/=4) error stop 'Unexpected beam buffer extent'
    allocate(value(n,m))
  end subroutine
end module
module B18F_MY_DEALLOC_MOD
contains
  subroutine B18F_MY_DEALLOC(value)
    real(8),allocatable,intent(inout) :: value(:,:)
    deallocate(value)
  end subroutine
end module
module B18F_SIGEPS131PI_MOD
end module
