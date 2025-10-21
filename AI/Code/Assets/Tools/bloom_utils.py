"""Utilities for loading secret lookups via Bloom filters."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Iterable

try:  # pragma: no cover - optional dependency
    from pybloom_live import BloomFilter  # type: ignore
except Exception:  # pragma: no cover - gracefully degrade
    BloomFilter = None  # type: ignore

LOGGER = logging.getLogger(__name__)


class SecretLookup:
    """Small helper that wraps a Bloom filter or set."""

    def __init__(self, secrets_path: Path) -> None:
        self.secrets_path = secrets_path
        self._filter = self._load(secrets_path)

    def _load(self, path: Path):
        if BloomFilter:
            try:
                return self._load_bloom(path)
            except Exception as exc:  # pragma: no cover - degrade gracefully
                LOGGER.warning("Bloom filter load failed (%s); using in-memory set", exc)
        return set(self._read_secrets(path))

    def _load_bloom(self, path: Path):  # pragma: no cover - depends on optional lib
        secrets = list(self._read_secrets(path))
        bloom = BloomFilter(capacity=max(len(secrets), 1), error_rate=0.001)
        for secret in secrets:
            bloom.add(secret)
        return bloom

    def _read_secrets(self, path: Path) -> Iterable[str]:
        if not path.exists():
            LOGGER.warning("Secrets file %s missing; using empty lookup", path)
            return []
        for line in path.read_text().splitlines():
            cleaned = line.strip()
            if cleaned and not cleaned.startswith("#"):
                yield cleaned

    def contains(self, candidate: str) -> bool:
        if hasattr(self._filter, "__contains__"):
            return candidate in self._filter  # type: ignore[operator]
        return False
