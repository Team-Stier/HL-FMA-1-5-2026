import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('capture', Path(__file__).resolve().parents[1] / 'scripts/capture_csv.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class CaptureTest(unittest.TestCase):
    def test_raw_frame_validation(self):
        self.assertEqual(capture.parse_raw_can_line('@CAN,1,0x102,2,ab01'),
                         ('1', '0x102', '2', 'AB01'))
        for line in ('@CAN,1,0x800,0,', '@CAN,1,0x102,8,0102', 'garbage'):
            self.assertIsNone(capture.parse_raw_can_line(line))

    def test_capture_without_startup_header_and_clean_interrupt(self):
        class Serial:
            def open(self):
                assert self.baudrate == 115200
                assert self.dtr is False and self.rts is False
                assert self.exclusive is True
                self.lines = iter([b'partial startup\n', b'@CAN,10,0x100,2,0102\n',
                                   b'@CAN,11,0x100,8,00\n',
                                   (','.join(['20', '1', '4'] + ['0'] * 24) + '\n').encode()])
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def readline(self):
                try: return next(self.lines)
                except StopIteration: raise KeyboardInterrupt
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(sys.modules, {'serial': types.SimpleNamespace(Serial=Serial, SerialException=OSError)}), \
                    patch.object(sys, 'argv', ['capture', '--port', '/dev/test', '--output-root', tmp]):
                self.assertEqual(capture.main(), 0)
            run = next(Path(tmp).iterdir())
            metadata = json.loads((run / 'run_metadata.json').read_text())
            self.assertEqual(metadata['row_count'], 1)
            self.assertEqual(metadata['can_frame_count'], 1)
            self.assertEqual(metadata['invalid_can_frame_line_count'], 1)
            self.assertEqual(metadata['stop_reason'], 'user_ctrl_c')
            self.assertEqual(len((run / 'raw_can.csv').read_text().splitlines()), 2)
            self.assertIn('0x100,2,0102', (run / 'can_frames.csv').read_text())


if __name__ == '__main__':
    unittest.main()
