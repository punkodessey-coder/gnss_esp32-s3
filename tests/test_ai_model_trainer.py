import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("trainer", Path(__file__).parents[1] / "scripts/train_ai_model.py")
trainer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trainer)


class TrainerTests(unittest.TestCase):
    def test_train_uses_only_normal_fix_rows_and_gnss_features(self):
        with tempfile.NamedTemporaryFile("w", newline="", encoding="utf-8", delete=False) as f:
            fields = ["fix", "hdop", "sat_used", "gsv_snr_avg", "accel_magnitude",
                      "gyro_magnitude", "sat_visible", "anomaly"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for i in range(40):
                writer.writerow(dict(fix=1, hdop=1.1 + i*.001, sat_used=8,
                                     gsv_snr_avg=35+i*.01, accel_magnitude=9.8,
                                     gyro_magnitude=.2, sat_visible=10, anomaly=0))
            # No-fix rows and anomalous fix rows must never contaminate the profile.
            for _ in range(50):
                writer.writerow(dict(fix=0, hdop=99.99, sat_used=0, gsv_snr_avg=8,
                                     accel_magnitude=10.7, gyro_magnitude=0.2,
                                     sat_visible=12, anomaly=0))
            for _ in range(10):
                writer.writerow(dict(fix=1, hdop=50, sat_used=1, gsv_snr_avg=5,
                                     accel_magnitude=10.7, gyro_magnitude=3.5,
                                     sat_visible=2, anomaly=1))
            path = f.name
        try:
            rows, centers, scales, mask = trainer.train(path)
            self.assertEqual(rows, 40)
            self.assertEqual(mask, 39)  # HDOP, used, C/N0, visible; no IMU
            self.assertEqual(centers[3:5], [0, 0])
            self.assertIn("trainingSamples = 40", trainer.render(rows, centers, scales, mask))
        finally:
            Path(path).unlink()

    def test_too_few_normal_fix_rows_rejected(self):
        with tempfile.NamedTemporaryFile("w", newline="", encoding="utf-8", delete=False) as f:
            fields = ["fix", "hdop", "sat_used", "gsv_snr_avg", "sat_visible", "anomaly"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for _ in range(40):
                writer.writerow(dict(fix=0, hdop=99.99, sat_used=0,
                                     gsv_snr_avg=30, sat_visible=10, anomaly=0))
            path = f.name
        try:
            with self.assertRaises(ValueError):
                trainer.train(path)
        finally:
            Path(path).unlink()


if __name__ == "__main__":
    unittest.main()
