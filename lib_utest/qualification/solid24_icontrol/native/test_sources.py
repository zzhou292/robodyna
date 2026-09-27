"""Small tamper/reversal checks for the native reference source admission."""
from pathlib import Path
import importlib.util,unittest,re
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("ic1_source",HERE/"prepare_sources.py");source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
class Sources(unittest.TestCase):
    def test_complete_authenticated_namespace_reversal(self):
        meta,values=source.generated();self.assertEqual(len(values),19)
        for row in meta["owned_native_sources"]:
            if not row.get("compile",True):continue
            raw=(source.ROOT/row["path"]).read_text();name=Path(row["source"]).name
            prepared=values[name]
            if name=="shour_ctl.F90":
                first=prepared.index("\n          call IC1_NATIVE_HOUR_WORK(")
                end=prepared.index("\n",prepared.index("hy4(i)*hgy4(i) )",first))+1
                prepared=prepared[:first]+prepared[end:]
            self.assertEqual(prepared.replace("IC1_NATIVE_","").replace("HEPH_NATIVE_",""),raw)
    def test_same_length_numerical_source_tamper_rejected(self):
        meta,_=source.verified();row=meta["owned_native_sources"][1];raw=(source.ROOT/row["path"]).read_bytes()
        index=raw.index(b"qh = one");changed=raw[:index]+b"qh = two"+raw[index+8:]
        self.assertEqual(len(changed),len(raw))
        with self.assertRaises(AssertionError):source.verify_bytes(changed,row)
    def test_foreign_length_and_blob_identity_rejected(self):
        meta,_=source.verified();row=meta["owned_native_sources"][0];raw=(source.ROOT/row["path"]).read_bytes()
        with self.assertRaises(AssertionError):source.verify_bytes(raw+b"\n",row)
        changed=dict(row,git_blob_sha1="0"*40)
        with self.assertRaises(AssertionError):source.verify_bytes(raw,changed)
if __name__=="__main__":unittest.main()
