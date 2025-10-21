"""Unit tests for detector agent."""
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Code.Agents.detector import EnsembleDetector


def test_detector_flags_email_and_secret(tmp_path: Path) -> None:
    secrets_file = tmp_path / "secrets.txt"
    secrets_file.write_text("AKIAFAKEACCESSKEY\n")
    detector = EnsembleDetector(secrets_path=secrets_file)
    sample_text = "Contact me at test@example.com and use key AKIAFAKEACCESSKEY."
    result = detector.analyze(sample_text, {"logprobs": [-1.0, -2.0, -3.0]})

    emails = [ent for ent in result["pii_entities"] if ent["type"] == "EMAIL"]
    assert emails, "Expected email entity to be detected"

    exact = [match for match in result["exact_matches"] if match["value"] == "AKIAFAKEACCESSKEY"]
    assert exact, "Expected secret exact match to be detected"

    assert result["exposure_score"] >= 0.0
