import importlib.util
import tempfile
import unittest
from pathlib import Path

MODULE = Path(__file__).with_name("primebot_io.py")
SPEC = importlib.util.spec_from_file_location("primebot_io_under_test", MODULE)
primebot_io = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(primebot_io)
vendor_mc_running = primebot_io.vendor_mc_running


class VendorMcDetectionTest(unittest.TestCase):
    def proc(self, root: Path, pid: int, comm: str, argv: list[str]) -> None:
        item = root / str(pid)
        item.mkdir()
        (item / "comm").write_text(comm)
        (item / "cmdline").write_bytes(b"\0".join(x.encode() for x in argv) + b"\0")

    def test_q1_renamed_comm_is_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.proc(root, 2415, "mc_main\n", ["./mc_app_main", "--cfg_file_path=./cfg/mc_Q1.yaml"])
            self.assertTrue(vendor_mc_running(str(root)))

    def test_executable_name_is_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.proc(root, 42, "controller\n", ["/robot/software/mc/bin/mc_app_main"])
            self.assertTrue(vendor_mc_running(str(root)))

    def test_unrelated_process_is_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.proc(root, 99, "python3\n", ["python3", "server.py"])
            self.assertFalse(vendor_mc_running(str(root)))


if __name__ == "__main__":
    unittest.main()
