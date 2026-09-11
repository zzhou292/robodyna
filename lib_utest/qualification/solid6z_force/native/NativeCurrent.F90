! SPDX-License-Identifier: AGPL-3.0-or-later
! Storage-only adapter around complete native current-geometry routines.
subroutine SOLID6Z_FORCE_CURRENT(G,D,REFERENCE_JACOBIAN,DT,STATUS)
  use SOLID6Z_FORCE_PACKETS
  use SOLID6Z_FORCE_ALEANIM_MOD
  use SOLID6Z_FORCE_MESSAGE_MOD
  use SOLID6Z_FORCE_S6ZDERITO3_MOD
  use SOLID6Z_FORCE_S6ZDEFOT3_MOD
  use SOLID6Z_FORCE_S6ZDERI3_MOD
  use SOLID6Z_FORCE_S6ZDEFC3_MOD
  use SOLID6Z_FORCE_S6ZDEFO3_MOD
  implicit none
  type(native_geometry),intent(inout) :: G
  real(kind=8),intent(in) :: D(MVSIZ,6,3),REFERENCE_JACOBIAN(10),DT
  integer,intent(out) :: STATUS
  real(kind=8) :: JAC_I(10,1),DET(MVSIZ),SPIN(MVSIZ,3),AXES(MVSIZ,9),GAMA(MVSIZ,6)
  real(kind=8) :: OFF(MVSIZ),OFFG(MVSIZ),OFFS(MVSIZ),VOLDP(MVSIZ),UNUSED_VZL(MVSIZ)
  real(kind=8) :: SAVED(1,15),NORMALS(MVSIZ,6,3),CORRECTED(MVSIZ,9)
  integer :: NGL(1)
  type(FANI_CELL_) :: CELL
  STATUS=0
  JAC_I(:,1)=REFERENCE_JACOBIAN
  OFF=1
  OFFG=1
  OFFS=1
  NGL=1
  CORRECTED=0
  SPIN=0
  GAMA=0
  SAVED=0
  call SOLID6Z_FORCE_S6ZDERITO3( &
    DET, G%P(:,1,1), G%P(:,2,1), G%P(:,3,1), &
    G%P(:,4,1), G%P(:,5,1), G%P(:,6,1), G%P(:,1,2), &
    G%P(:,2,2), G%P(:,3,2), G%P(:,4,2), G%P(:,5,2), &
    G%P(:,6,2), G%P(:,1,3), G%P(:,2,3), G%P(:,3,3), &
    G%P(:,4,3), G%P(:,5,3), G%P(:,6,3), 1, &
    JAC_I)
  call SOLID6Z_FORCE_S6ZDEFOT3( &
    G%P(:,1,1), G%P(:,2,1), G%P(:,3,1), G%P(:,4,1), &
    G%P(:,5,1), G%P(:,6,1), G%P(:,1,2), G%P(:,2,2), &
    G%P(:,3,2), G%P(:,4,2), G%P(:,5,2), G%P(:,6,2), &
    G%P(:,1,3), G%P(:,2,3), G%P(:,3,3), G%P(:,4,3), &
    G%P(:,5,3), G%P(:,6,3), D(:,1,1), D(:,2,1), &
    D(:,3,1), D(:,4,1), D(:,5,1), D(:,6,1), &
    D(:,1,2), D(:,2,2), D(:,3,2), D(:,4,2), &
    D(:,5,2), D(:,6,2), D(:,1,3), D(:,2,3), &
    D(:,3,3), D(:,4,3), D(:,5,3), D(:,6,3), &
    G%WORLD_GRADIENT(:,1), G%WORLD_GRADIENT(:,2), G%WORLD_GRADIENT(:,3), G%WORLD_GRADIENT(:,4), &
    G%WORLD_GRADIENT(:,5), G%WORLD_GRADIENT(:,6), G%WORLD_GRADIENT(:,7), G%WORLD_GRADIENT(:,8), &
    G%WORLD_GRADIENT(:,9), SPIN(:,1), SPIN(:,2), SPIN(:,3), &
    1)
  call SOLID6Z_FORCE_SZTORTH3( &
    1, 1, 0, 1, &
    G%FRAME(:,1), G%FRAME(:,2), G%FRAME(:,3), G%FRAME(:,4), &
    G%FRAME(:,5), G%FRAME(:,6), G%FRAME(:,7), G%FRAME(:,8), &
    G%FRAME(:,9), AXES(:,1), AXES(:,2), AXES(:,3), &
    AXES(:,4), AXES(:,5), AXES(:,6), AXES(:,7), &
    AXES(:,8), AXES(:,9), GAMA)
  G%MATERIAL_GRADIENT=G%WORLD_GRADIENT
  call SOLID6Z_FORCE_SORDEFT3( &
    1, 1, G%MATERIAL_GRADIENT(:,1), G%MATERIAL_GRADIENT(:,2), &
    G%MATERIAL_GRADIENT(:,3), G%MATERIAL_GRADIENT(:,4), G%MATERIAL_GRADIENT(:,5), G%MATERIAL_GRADIENT(:,6), &
    G%MATERIAL_GRADIENT(:,7), G%MATERIAL_GRADIENT(:,8), G%MATERIAL_GRADIENT(:,9), AXES(:,1), &
    AXES(:,2), AXES(:,3), AXES(:,4), AXES(:,5), &
    AXES(:,6), AXES(:,7), AXES(:,8), AXES(:,9))
  call SOLID6Z_FORCE_S6ZDERI3( &
    OFF, DET, NGL, G%X(:,1,1), &
    G%X(:,2,1), G%X(:,3,1), G%X(:,4,1), G%X(:,5,1), &
    G%X(:,6,1), G%X(:,1,2), G%X(:,2,2), G%X(:,3,2), &
    G%X(:,4,2), G%X(:,5,2), G%X(:,6,2), G%X(:,1,3), &
    G%X(:,2,3), G%X(:,3,3), G%X(:,4,3), G%X(:,5,3), &
    G%X(:,6,3), G%P(:,1,1), G%P(:,2,1), G%P(:,3,1), &
    G%P(:,4,1), G%P(:,5,1), G%P(:,6,1), G%P(:,1,2), &
    G%P(:,2,2), G%P(:,3,2), G%P(:,4,2), G%P(:,5,2), &
    G%P(:,6,2), G%P(:,1,3), G%P(:,2,3), G%P(:,3,3), &
    G%P(:,4,3), G%P(:,5,3), G%P(:,6,3), G%JACOBIAN(:,1), &
    G%JACOBIAN(:,2), G%JACOBIAN(:,3), G%JACOBIAN(:,4), G%JACOBIAN(:,5), &
    G%JACOBIAN(:,6), G%JACOBIAN(:,9), UNUSED_VZL, G%VOLUME, &
    SAVED, OFFG, 1, 10, &
    VOLDP, 1)
  if (OFF(1)/=1 .or. OFFG(1)/=1 .or. GEOMETRY_ERRORS/=0) then
    STATUS=1
    return
  end if
! VZL is unused by S6ZFORC3; the complete donor's uninitialized auxiliary is
! intentionally neither read nor exposed by this selected caller.
  call SOLID6Z_FORCE_SDLEN3( &
    G%VOLUME, G%LENGTH, G%X(:,1,1), G%X(:,2,1), &
    G%X(:,3,1), G%X(:,3,1), G%X(:,4,1), G%X(:,5,1), &
    G%X(:,6,1), G%X(:,6,1), G%X(:,1,2), G%X(:,2,2), &
    G%X(:,3,2), G%X(:,3,2), G%X(:,4,2), G%X(:,5,2), &
    G%X(:,6,2), G%X(:,6,2), G%X(:,1,3), G%X(:,2,3), &
    G%X(:,3,3), G%X(:,3,3), G%X(:,4,3), G%X(:,5,3), &
    G%X(:,6,3), G%X(:,6,3), NORMALS(:,1,1), NORMALS(:,2,1), &
    NORMALS(:,3,1), NORMALS(:,4,1), NORMALS(:,5,1), NORMALS(:,6,1), &
    NORMALS(:,1,2), NORMALS(:,2,2), NORMALS(:,3,2), NORMALS(:,4,2), &
    NORMALS(:,5,2), NORMALS(:,6,2), NORMALS(:,1,3), NORMALS(:,2,3), &
    NORMALS(:,3,3), NORMALS(:,4,3), NORMALS(:,5,3), NORMALS(:,6,3), &
    1, 42, 0, 0)
  call SOLID6Z_FORCE_S6ZDEFC3( &
    G%P(:,1,1), G%P(:,2,1), G%P(:,3,1), G%P(:,4,1), &
    G%P(:,5,1), G%P(:,6,1), G%P(:,1,2), G%P(:,2,2), &
    G%P(:,3,2), G%P(:,4,2), G%P(:,5,2), G%P(:,6,2), &
    G%P(:,1,3), G%P(:,2,3), G%P(:,3,3), G%P(:,4,3), &
    G%P(:,5,3), G%P(:,6,3), G%V(:,1,1), G%V(:,2,1), &
    G%V(:,3,1), G%V(:,4,1), G%V(:,5,1), G%V(:,6,1), &
    G%V(:,1,2), G%V(:,2,2), G%V(:,3,2), G%V(:,4,2), &
    G%V(:,5,2), G%V(:,6,2), G%V(:,1,3), G%V(:,2,3), &
    G%V(:,3,3), G%V(:,4,3), G%V(:,5,3), G%V(:,6,3), &
    G%VELOCITY_GRADIENT(:,1), G%VELOCITY_GRADIENT(:,2), G%VELOCITY_GRADIENT(:,3), G%VELOCITY_GRADIENT(:,4), &
    G%VELOCITY_GRADIENT(:,5), G%VELOCITY_GRADIENT(:,6), G%VELOCITY_GRADIENT(:,7), G%VELOCITY_GRADIENT(:,8), &
    G%VELOCITY_GRADIENT(:,9), SPIN(:,1), SPIN(:,2), SPIN(:,3), &
    1)
  call SOLID6Z_FORCE_S6ZDEFO3( &
    CORRECTED(:,1), CORRECTED(:,2), CORRECTED(:,3), CORRECTED(:,4), &
    CORRECTED(:,5), CORRECTED(:,6), CORRECTED(:,7), CORRECTED(:,8), &
    CORRECTED(:,9), G%RATE(:,4), G%RATE(:,5), G%RATE(:,6), &
    G%VELOCITY_GRADIENT(:,1), G%VELOCITY_GRADIENT(:,2), G%VELOCITY_GRADIENT(:,3), G%VELOCITY_GRADIENT(:,4), &
    G%VELOCITY_GRADIENT(:,5), G%VELOCITY_GRADIENT(:,6), G%VELOCITY_GRADIENT(:,7), G%VELOCITY_GRADIENT(:,8), &
    G%VELOCITY_GRADIENT(:,9), G%VOLUME, OFF, OFFG, &
    OFFS, VOLDP, 1, DT, &
    0, 0, 1, 10, &
    SPIN(:,1), SPIN(:,2), SPIN(:,3), 0, &
    0, CELL, 1)
  G%RATE(:,1)=CORRECTED(:,1)
  G%RATE(:,2)=CORRECTED(:,5)
  G%RATE(:,3)=CORRECTED(:,9)
  if (OFF(1)/=1) STATUS=2
end subroutine
