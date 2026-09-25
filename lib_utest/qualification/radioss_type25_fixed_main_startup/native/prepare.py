#!/usr/bin/env python3
"""Prepare independently pinned native startup/ready topology and float stages."""
import argparse
from pathlib import Path
from Sources import ROOT,read,constants
from Extraction import routines,namespace,csr_blocks


def generated():
    source=read();original=routines(source)
    result={name:namespace(text) for name,text in original.items()}
    normal=result["StarterNormals.F"]
    point="      REM30 = RUN/REP30\n"
    assert normal.count(point)==1
    result["StarterNormals.F"]=normal.replace(point,point+
      "      observed_rep30=REP30\n      observed_rem30=REM30\n")
    ready=result["ReadyNormals.F"]
    ready=ready.replace("      USE STARTUP_NATIVE_MPI\n", "      USE STARTUP_NATIVE_MPI\n"
      "      USE STARTUP_NATIVE_NORMAL_STORAGE, ONLY: observed_ready_rep30, observed_ready_rem30\n")
    point="      IF(FLAG == 1) THEN\n"
    assert ready.count(point)==1
    ready=ready.replace(point,point+
      "      observed_ready_rep30=REP30\n      observed_ready_rem30=REM30\n")
    assert ready.count('"mvsiz_p.inc"')==1
    result["ReadyNormals.F"]=ready.replace('"mvsiz_p.inc"','"engine_mvsiz_p.inc"')
    result["Constants.F90"]=constants(source["constant_mod.F"],original.values()).replace(
      "module selection_constants","module startup_native_constants")
    references,csr=csr_blocks(source)
    wrapper=(ROOT/"CsrWrapper.F.in").read_text()
    for name,value in [("REFERENCES",references),("CSR",csr)]:
        assert wrapper.count("@"+name+"@")==1
        wrapper=wrapper.replace("@"+name+"@",value)
    result["CsrWrapper.F"]=wrapper
    for name in ["Memory.F90","Boundary.F90","NormalStorage.F90","Wrapper.F90"]:
        result[name]=(ROOT/name).read_text()
    result["implicit_f.inc"]=("      USE ISO_C_BINDING\n      USE STARTUP_NATIVE_CONSTANTS\n"
      "      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n")
    # Preserve the actual selected declarations: Starter512 and GNU/Linux
    # Engine129. NVSIZ128 is a different native parameter, never substituted.
    starter=source["starter_mvsiz_p.inc"]
    declaration="       INTEGER MVSIZ\n       PARAMETER (MVSIZ = 512)\n"
    assert starter.count(declaration)==1
    result["mvsiz_p.inc"]=declaration
    engine=source["engine_mvsiz_p.inc"]
    branch=next(line for line in engine.splitlines() if line.startswith("#elif CPP_mach == CPP_linux64_spmd"))
    tail=engine.split(branch+"\n",1)[1]
    selected="      PARAMETER (MVSIZ = 129)\n"
    assert tail.startswith(selected)
    result["engine_mvsiz_p.inc"]="       INTEGER MVSIZ\n"+selected
    result["com01_c.inc"]="      INTEGER NSPMD,NINTER25\n      COMMON /STARTUP_NATIVE_PARTITION/NSPMD,NINTER25\n"
    result["com04_c.inc"]="      INTEGER NUMNOD,NUMELS\n      COMMON /STARTUP_NATIVE_COUNTS/NUMNOD,NUMELS\n"
    result["task_c.inc"]="      INTEGER NTHREAD\n      COMMON /STARTUP_NATIVE_THREADS/NTHREAD\n"
    result["param_c.inc"]="      INTEGER,PARAMETER::NLEDGE=15,NPROPGI=1,LNOPT1=1\n"
    result["units_c.inc"]="      INTEGER,PARAMETER::ISTDO=6,IOUT=6\n"
    result["scr03_c.inc"]="      INTEGER,PARAMETER::IPRI=1\n"
    result["scr17_c.inc"]=""
    result["vectorize.inc"]=""
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);parser.add_argument("--check",action="store_true")
    args=parser.parse_args()
    for name,text in generated().items():
        if args.output:
            target=args.output/name
            if args.check:assert target.read_text()==text
            else:target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
    print("Pinned native side, topology, CSR and distinct float-normal stages prepared")
