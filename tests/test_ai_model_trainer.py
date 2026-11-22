import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("trainer", Path(__file__).parents[1] / "scripts/train_ai_model.py")
trainer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trainer)

class TrainerTests(unittest.TestCase):
    def test_train_and_render(self):
        with tempfile.NamedTemporaryFile("w", newline="", encoding="utf-8", delete=False) as f:
            fields = ["fix", "hdop", "sat_used", "gsv_snr_avg", "accel_magnitude", "gyro_magnitude", "sat_visible", "anomaly"]
            writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
            for i in range(40):
                writer.writerow(dict(fix=1, hdop=1.1 + i*.001, sat_used=8, gsv_snr_avg=35+i*.01,
                                     accel_magnitude=9.8, gyro_magnitude=.2, sat_visible=10, anomaly=0))
            path = f.name
        try:
            rows, centers, scales, mask = trainer.train(path)
            self.assertEqual(rows, 40)
            self.assertEqual(mask, 63)
            self.assertIn("enabled = true", trainer.render(rows, centers, scales, mask))
        finally:
            Path(path).unlink()

    def test_too_few_rows_rejected(self):
        with tempfile.NamedTemporaryFile("w", newline="", encoding="utf-8", delete=False) as f:
            fields = ["fix", "hdop", "sat_used", "gsv_snr_avg", "accel_magnitude", "gyro_magnitude", "sat_visible", "anomaly"]
            writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
            writer.writerow(dict(fix=1, hdop=1, sat_used=8, gsv_snr_avg=30, accel_magnitude=9.8, gyro_magnitude=.2, sat_visible=10, anomaly=0))
            path=f.name
        try:
            with self.assertRaises(ValueError): trainer.train(path)
        finally: Path(path).unlink()

if __name__ == "__main__": unittest.main()
