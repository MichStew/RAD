"""Tests for policy reasoner logic."""
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Code.Agents.policy_reasoner import PolicyReasoner


def test_high_severity_triggers_redact(tmp_path: Path) -> None:
    policy_file = tmp_path / "policy.json"
    policy_file.write_text(
        """
        {
            "weights": {"exact": 0.6, "pii": 0.3, "exposure": 0.1},
            "thresholds": {"redact": 0.5, "paraphrase": 0.2, "pass": 0.0},
            "default_action": "PASS"
        }
        """
    )
    reasoner = PolicyReasoner(policy_path=policy_file)
    detection = {
        "exact_matches": [{"value": "AKIA", "severity": 0.9}],
        "pii_entities": [],
        "exposure_score": 0.1,
    }

    decision = reasoner.decide(detection)
    assert decision["action"] in {"REDACT", "REFUSE", "ESCALATE"}
