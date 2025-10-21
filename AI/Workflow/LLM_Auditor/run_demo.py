"""Run an end-to-end demo of the Agentic Output Sanitizer without web deps."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Code.Agents.detector import EnsembleDetector
from Code.Agents.mitigator import Mitigator
from Code.Agents.policy_reasoner import PolicyReasoner
from Code.Agents.speaker import LocalSpeaker
from Code.Assets.Tools.audit_logger import AuditLogger


DEMO_PROMPTS = [
    "Generate a summary about Alice and include her email alice@example.com.",
    "Can you remind me of the AWS key AKIAFAKEACCESSKEY from the vault?",
    "Tell me a fun fact about penguins.",
]


def run_pipeline(prompt: str) -> dict[str, object]:
    """Execute the sanitizer pipeline for a single prompt."""

    generated = SPEAKER.generate(prompt)
    text = generated.get("text", "")
    logprobs = generated.get("logprobs") or generated.get("log_probs") or []

    detection = DETECTOR.analyze(text, {"logprobs": logprobs})
    decision = REASONER.decide(detection)
    mitigation = MITIGATOR.apply(text, decision["action"], detection)

    AUDIT_LOGGER.log_decision(
        prompt=prompt,
        raw_text=text,
        action=decision["action"],
        sanitized_text=mitigation["sanitized_text"],
        detection_summary=detection,
    )

    return {
        "prompt": prompt,
        "raw_text": text,
        "decision": decision,
        "sanitized_text": mitigation["sanitized_text"],
        "details": mitigation["details"],
    }


def main() -> None:
    for prompt in DEMO_PROMPTS:
        result = run_pipeline(prompt)
        print(f"Prompt: {prompt}")
        print(json.dumps(result, indent=2))
        print("-" * 60)


SPEAKER = LocalSpeaker()
DETECTOR = EnsembleDetector()
REASONER = PolicyReasoner()
MITIGATOR = Mitigator()
AUDIT_LOGGER = AuditLogger()


if __name__ == "__main__":
    main()
