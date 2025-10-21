"""PII detection helpers with spaCy or regex fallback."""
from __future__ import annotations

import logging
import re
from typing import Dict, List

LOGGER = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency
    import spacy
    _NLP = spacy.load("en_core_web_sm")
except Exception:  # pragma: no cover - fallback
    _NLP = None
    LOGGER.warning("spaCy model unavailable; using regex-based NER fallback")

_EMAIL_RE = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
_PHONE_RE = re.compile(r"\b(?:\d[ -]?){7,}\b")


def extract_pii_entities(text: str) -> List[Dict[str, object]]:
    """Extract simple PII entities from text."""
    entities: List[Dict[str, object]] = []
    if _NLP is not None:  # pragma: no cover - requires spaCy model
        doc = _NLP(text)
        for ent in doc.ents:
            if ent.label_ in {"PERSON", "GPE", "LOC", "ORG", "EMAIL", "PHONE"}:
                entities.append({"type": ent.label_, "span": ent.text, "conf": 0.9})
        return entities

    for match in _EMAIL_RE.finditer(text):
        entities.append({"type": "EMAIL", "span": match.group(0), "conf": 0.85})
    for match in _PHONE_RE.finditer(text):
        span = match.group(0)
        if any(char.isdigit() for char in span):
            entities.append({"type": "PHONE", "span": span, "conf": 0.75})
    return entities
