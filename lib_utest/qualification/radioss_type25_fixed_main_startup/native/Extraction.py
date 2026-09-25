"""Complete native routines and exact retained neighborhood/reference blocks."""
from Sources import routine,between


def routines(source):
    result={"Expand.F":routine(source["i25surfi.F"],"SH2SURF25"),
      "StarterNormals.F":routine(source["i25norm3.F"],"I25NORM"),
      "ReadyNormals.F":routine(source["i25norm.F"],"I25NORMP"),
      "FreeMain.F":routine(source["i25free_bound.F"],"I25FREE_BOUND")}
    for name in ["I25NEIGH_SEG_EN","I25NEIGH_SEG_E","I25NEIGH_REMOVEALLBUT1","I25NEIGH_SEG_OPP","I25NEIGH_MSG_ERR"]:
        result[name+".F"]=routine(source["i25neigh.F"],name)
    for name in ["ADD_ID","INTAB","SAME_SEG","NORMV3"]:
        result[name+".F"]=routine(source["i24tools.F"],name)
    result["NORMA4N.F"]=routine(source["norma1.F"],"NORMA4N")
    # Every retained topology/reference/boundary output is complete before this
    # exact source cut. The omitted tail only builds edge-contact/subsurface
    # tables, outside the IEDGE0/NISUB0 profile. No retained output is repaired.
    neigh=routine(source["i25neigh.F"],"I25NEIGH")
    stop='      call my_alloc(CLEF, 4, NEDGE, "CLEF")\n'
    assert neigh.count(stop)==1
    result["Neighborhood.F"]=neigh.split(stop,1)[0]+"      CALL MY_DEALLOC(LEDGE_TMP1)\n      RETURN\n      END\n"
    return result


def namespace(text):
    return text.replace("USE MY_ALLOC_MOD","USE STARTUP_NATIVE_MEMORY").replace(
      "use my_alloc_mod","use startup_native_memory").replace(
      "use my_dealloc_mod, only : my_dealloc","use startup_native_memory, only : my_dealloc").replace(
      "USE MOD_I25NORM","USE STARTUP_NATIVE_NORMAL_STORAGE").replace(
      "USE MESSAGE_MOD","USE STARTUP_NATIVE_MESSAGES").replace(
      "USE NAMES_AND_TITLES_MOD","USE STARTUP_NATIVE_NAMES").replace(
      "USE MPI_COMMOD","USE STARTUP_NATIVE_MPI").replace(
      "CALL MY_BARRIER","CALL STARTUP_NATIVE_BARRIER").replace(
      "CALL SPMD_EXCH_NOR","CALL STARTUP_NATIVE_FOREIGN_EXCHANGE").replace(
      "call spmd_exch_nor","call startup_native_foreign_exchange")


def csr_blocks(source):
    body=routine(source["inter_tools.F"],"PREPARE_SPLIT_I25")
    references=between(body,"      NADMSR_L=0\n","      CNMN_L = 0 ")
    start="      DO I=1,NRTM\n        IF(INTERCEP%P(I)==PROC+1)THEN\n          DO K=1,3\n"
    assert body.count(start)==2
    csr=body[body.index(start):].split("      RETURN\n",1)[0]
    return references,csr
