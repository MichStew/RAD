"""Apply mitigation strategies to sanitize model outputs."""
from __future__ import annotations

import logging
from typing import Dict

from Code.Assets.Tools.paraphrase_wrapper import Paraphraser

LOGGER = logging.getLogger(__name__)


class Mitigator:
    """Implement mitigation actions requested by the policy reasoner."""

    def __init__(self) -> None:
        self.paraphraser = Paraphraser()

    def apply(self, text: str, action: str, detection: Dict[str, object]) -> Dict[str, object]:
        """Apply the chosen mitigation action to the text."""
        if action == "PASS":
            return {"sanitized_text": text, "details": {"action": action, "note": "No sanitization required."}}
        if action == "REFUSE":
            refusal = "I’m sorry, but I can’t share that information."
            return {"sanitized_text": refusal, "details": {"action": action, "note": "Refusal per policy."}}
        if action == "ESCALATE":
            message = "Content flagged for human review."
            return {"sanitized_text": message, "details": {"action": action, "note": "Escalated to compliance."}}
        if action == "PARAPHRASE":
            paraphrased = self.paraphraser.paraphrase(text)
            return {"sanitized_text": paraphrased, "details": {"action": action, "note": "Paraphrased output."}}
        if action == "REDACT":
            redacted = self._redact(text, detection)
            return {"sanitized_text": redacted, "details": {"action": action, "note": "Sensitive spans redacted."}}

        LOGGER.warning("Unknown action %s - returning original text", action)
        return {"sanitized_text": text, "details": {"action": "UNKNOWN", "note": "Action not recognized."}}

    def _redact(self, text: str, detection: Dict[str, object]) -> str:
        redacted_text = text
        for match in detection.get("exact_matches", []):  # type: ignore[assignment]
            value = match.get("value")
            if value:
                redacted_text = redacted_text.replace(value, "[REDACTED]")
        for entity in detection.get("pii_entities", []):  # type: ignore[assignment]
            span = entity.get("span")
            if span:
                redacted_text = redacted_text.replace(span, f"[{entity.get('type', 'PII')}]")
        return redacted_text
