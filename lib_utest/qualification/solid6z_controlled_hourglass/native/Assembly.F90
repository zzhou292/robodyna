! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine S6_CONTROL_ASSEMBLE(F,STI,FORCES,NODAL)
  use SOLID6Z_FORCE_PACKETS
  implicit none
  real(kind=8),intent(inout)::F(MVSIZ,6,3),STI(MVSIZ)
  real(kind=8),intent(out)::FORCES(18),NODAL(6)
  real(kind=8)::OFF(MVSIZ),ASSEMBLED(3,6),THEM(MVSIZ,6),FTHE(1),CONDN(1),CONDE(MVSIZ)
  integer::NODES(1,6),N,K
  OFF=1;ASSEMBLED=0;NODAL=0;THEM=0;FTHE=0;CONDN=0;CONDE=0
  do N=1,6
    NODES(1,N)=N
  enddo
  call S6_CONTROL_ASSEMBLY_NATIVE(OFF,ASSEMBLED, &
    NODES(1,1),NODES(1,2),NODES(1,3),NODES(1,4),NODES(1,5),NODES(1,6), &
    NODAL,STI,F(1,1,1),F(1,1,2),F(1,1,3),F(1,2,1),F(1,2,2),F(1,2,3), &
    F(1,3,1),F(1,3,2),F(1,3,3),F(1,4,1),F(1,4,2),F(1,4,3), &
    F(1,5,1),F(1,5,2),F(1,5,3),F(1,6,1),F(1,6,2),F(1,6,3), &
    1,0,FTHE,THEM,CONDN,CONDE,1,1,0)
  do N=1,6
    do K=1,3
      FORCES(3*(N-1)+K)=ASSEMBLED(K,N)
    enddo
  enddo
end subroutine
