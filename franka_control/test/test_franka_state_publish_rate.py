from pathlib import Path
import re
import unittest


CONFIG = Path(__file__).parents[1] / "config" / "default_controllers.yaml"


class FrankaStatePublishRateTest(unittest.TestCase):
    def test_operational_state_controller_publishes_at_200_hz(self):
        text = CONFIG.read_text(encoding="utf-8")
        match = re.search(
            r"(?ms)^franka_state_controller:\s*$"
            r"(.*?)(?=^[A-Za-z_].*:\s*$|\Z)",
            text,
        )
        self.assertIsNotNone(match, "franka_state_controller block missing")
        rate = re.search(r"(?m)^\s+publish_rate:\s*(\d+)", match.group(1))
        self.assertIsNotNone(rate, "publish_rate missing")
        self.assertEqual(int(rate.group(1)), 200)


if __name__ == "__main__":
    unittest.main()
