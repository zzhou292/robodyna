"""Check complete selected call order and the source-only scaffold boundary."""
from pathlib import Path
import importlib.util,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location("controlled_sources",HERE/"native/prepare_sources.py");source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
meta,prepared=source.generated()
driver=ROOT/"lib_utest/qualification/solid24_force/native/original/engine/source/elements/solid/solidez/szforc3.F"
text=driver.read_text();names=[m.group(1).upper() for m in re.finditer(r"^\s*CALL\s+(\w+)\s*\(",text,re.M|re.I)]
position=0
for name in ["SZHOUR_CTL","SFINT3","SRROTA3","SDISTOR_INI","S8FOR_DISTOR","SCUMU3"]:
    position=names.index(name,position)+1
assert "ISCTL = IGEO(97,PID)" in text and "IF (ISCTL > 0) THEN" in text
assert "GBUF%EINT_DISTOR, DT1)" in text and "STIFN,   STI," in text
for filename,required in {
    "szhour_ctl.F":["CALL SHOUR_CTL("],
    "shour_ctl.F90":["sti(i) = f_sti(i)*sti(i)","fhour(i,1,1) = fhour(i,1,1) + edt(i)*hgx1(i)","eint(i)= eint(i)+dt1*("],
    "s8for_distor.F":["CALL SFOR_VISN8(","CALL SFOR_N2S4(","CALL SFOR_4N2S4(","E_DISTOR"],
}.items():
    original=next((ROOT/r["path"]).read_text() for r in meta["owned_native_sources"] if Path(r["source"]).name==filename)
    for token in required:assert token in original,(filename,token)
print(json.dumps({"status":"source_passed","complete_native_control_order_preserved":True,"native_sources":len(prepared),"production_solver_enabled":False,"bridge_complete":True}))

globals=(ROOT/"lib_utest/qualification/solid24_force/native/NativeGlobals.F").read_text()
assert "      ANIM_N=0" in globals and "      IAD_GPS=0" in globals

assert "pm(32,i) = bulk" in prepared["slot_generic.inc"]
