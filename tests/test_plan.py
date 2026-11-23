"""Exercise the published command line without a camera stream or external models."""
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest


class PlanTests(unittest.TestCase):
    def test_plan_reads_metadata_without_loading_assets_or_creating_output(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            processed = root / "processed"
            processed.mkdir()
            (processed / "camera.json").write_text(json.dumps({"frames": 3}), encoding="utf-8")
            episode = root / "episode"
            episode.mkdir()
            output = root / "planned result"
            command = [
                sys.executable, "-m", "grasp_retargeting", "plan",
                "--processed", str(processed), "--episode", str(episode),
                "--mano-model", str(root / "MANO_RIGHT.pkl"),
                "--hand", "leap_hand", "--output", str(output),
            ]
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("leap_hand", result.stdout)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
