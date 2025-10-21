"""Fuse detector signals with policy rules to pick mitigation actions."""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

LOGGER = logging.getLogger(__name__)


@dataclass
class Policy:
    weights: Dict[str, float]
    thresholds: Dict[str, float]
    default_action: str

    @classmethod
    def from_file(cls, path: Path) -> "Policy":
        data = json.loads(path.read_text())
        return cls(
            weights=data.get("weights", {}),
            thresholds=data.get("thresholds", {}),
            default_action=data.get("default_action", "PASS"),
        )


class PolicyReasoner:
    """Combine detector scores and policies to reach a final decision."""

    def __init__(self, policy_path: Path | None = None) -> None:
        tools_dir = Path(__file__).resolve().parents[2] / "Assets" / "Tools"
        policy_file = policy_path or tools_dir / "policy.json"
        self.policy = Policy.from_file(policy_file)

    def decide(self, detection: Dict[str, object], policy_override: Dict[str, object] | None = None) -> Dict[str, object]:
        """Decide on mitigation action based on detection results."""
        policy = self.policy
        if policy_override:
            merged = {
                "weights": {**policy.weights, **policy_override.get("weights", {})},
                "thresholds": {**policy.thresholds, **policy_override.get("thresholds", {})},
                "default_action": policy_override.get("default_action", policy.default_action),
            }
            policy = Policy(**merged)  # type: ignore[arg-type]

        exact_severity = max((item.get("severity", 0.0) for item in detection.get("exact_matches", [])), default=0.0)  # type: ignore[arg-type]
        pii_conf = max((item.get("conf", 0.0) for item in detection.get("pii_entities", [])), default=0.0)  # type: ignore[arg-type]
        exposure = float(detection.get("exposure_score", 0.0))

        weights = policy.weights or {"exact": 0.6, "pii": 0.3, "exposure": 0.1}
        risk = (
            weights.get("exact", 0.6) * exact_severity
            + weights.get("pii", 0.3) * pii_conf
            + weights.get("exposure", 0.1) * exposure
        )

        action = policy.default_action
        thresholds = policy.thresholds or {}
        # Evaluate from highest severity to lowest.
        ordered_actions = [
            ("ESCALATE", thresholds.get("escalate", 0.95)),
            ("REFUSE", thresholds.get("refuse", 0.85)),
            ("REDACT", thresholds.get("redact", 0.6)),
            ("PARAPHRASE", thresholds.get("paraphrase", 0.4)),
            ("PASS", thresholds.get("pass", 0.0)),
        ]
        rationale = ""
        for candidate, threshold in ordered_actions:
            if risk >= threshold:
                action = candidate
                rationale = f"risk {risk:.2f} >= {threshold:.2f} threshold for {candidate}"
                break

        if not rationale:
            rationale = "risk below configured thresholds"
        LOGGER.debug("Policy decision: action=%s risk=%.3f", action, risk)
        return {"action": action, "score": risk, "rationale": rationale}
