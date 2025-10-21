"""Ensemble detector for privacy-sensitive leakage."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Dict, List, Optional

from Code.Assets.Tools.bloom_utils import SecretLookup
from Code.Assets.Tools.ner_utils import extract_pii_entities

LOGGER = logging.getLogger(__name__)


class EnsembleDetector:
    """Combine multiple detectors to flag sensitive content."""

    def __init__(self, secrets_path: Optional[Path] = None) -> None:
        data_dir = Path(__file__).resolve().parents[2] / "Data"
        default_path = data_dir / "secrets.txt"
        self.lookup = SecretLookup(secrets_path or default_path)

    def analyze(self, text: str, generation_meta: Optional[Dict] = None) -> Dict[str, object]:
        """Analyze a text sample and return structured detection information."""

        exact_matches = self._detect_exact_matches(text)
        pii_entities = extract_pii_entities(text)
        exposure_score = self._heuristic_exposure(generation_meta)

        report = {
            "exact_matches": exact_matches,
            "pii_entities": pii_entities,
            "exposure_score": exposure_score,
        }
        LOGGER.debug("Detection report: %s", report)
        return report

    def _detect_exact_matches(self, text: str) -> List[Dict[str, float]]:
        matches: List[Dict[str, float]] = []
        for candidate in text.split():
            cleaned = candidate.strip().strip(",.;:\"'()[]{}")
            if len(cleaned) < 6:
                continue
            if self.lookup.contains(cleaned):
                matches.append({"value": cleaned, "severity": 0.99})
        return matches

    def _heuristic_exposure(self, generation_meta: Optional[Dict]) -> float:
        if not generation_meta:
            return 0.0
        logprobs = generation_meta.get("logprobs") or generation_meta.get("log_probs")
        if not logprobs:
            return 0.0
        mean_logprob = sum(logprobs) / max(len(logprobs), 1)
        # The more confident the model (higher logprob), the less risky. We
        # invert and normalise into [0,1] with a simple logistic style mapping.
        exposure = min(max(( -mean_logprob) / 10.0, 0.0), 1.0)
        return exposure
