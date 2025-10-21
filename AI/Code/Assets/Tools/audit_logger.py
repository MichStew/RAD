"""Audit logging utilities for orchestrator decisions."""
from __future__ import annotations

import json
import logging
import sqlite3
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

LOGGER = logging.getLogger(__name__)


@dataclass
class AuditRecord:
    timestamp: str
    prompt: str
    raw_text: str
    action: str
    sanitized_text: str
    detection_summary: Dict[str, Any]


class AuditLogger:
    """Persist audit trail to SQLite with JSON fallback."""

    def __init__(self, db_path: Path | None = None) -> None:
        base_dir = Path(__file__).resolve().parents[3]
        default_path = base_dir / "audit_log.sqlite"
        self.db_path = db_path or default_path
        self.json_path = self.db_path.with_suffix(".json")
        self._init_storage()

    def _init_storage(self) -> None:
        try:
            conn = sqlite3.connect(self.db_path)
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS audit_log (
                    timestamp TEXT,
                    prompt TEXT,
                    raw_text TEXT,
                    action TEXT,
                    sanitized_text TEXT,
                    detection_summary TEXT
                )
                """
            )
            conn.commit()
            conn.close()
            self._use_sqlite = True
        except sqlite3.Error as exc:  # pragma: no cover - fallback path
            LOGGER.warning("SQLite unavailable (%s); falling back to JSON log", exc)
            self._use_sqlite = False

    def log(self, record: AuditRecord) -> None:
        if getattr(self, "_use_sqlite", False):
            self._write_sqlite(record)
        else:
            self._write_json(record)

    def _write_sqlite(self, record: AuditRecord) -> None:
        conn = sqlite3.connect(self.db_path)
        conn.execute(
            "INSERT INTO audit_log VALUES (?, ?, ?, ?, ?, ?)",
            (
                record.timestamp,
                record.prompt,
                record.raw_text,
                record.action,
                record.sanitized_text,
                json.dumps(record.detection_summary),
            ),
        )
        conn.commit()
        conn.close()

    def _write_json(self, record: AuditRecord) -> None:
        existing: list[dict[str, Any]] = []
        if self.json_path.exists():
            existing = json.loads(self.json_path.read_text())
        existing.append(asdict(record))
        self.json_path.write_text(json.dumps(existing, indent=2))

    def log_decision(self, prompt: str, raw_text: str, action: str, sanitized_text: str, detection_summary: Dict[str, Any]) -> None:
        record = AuditRecord(
            timestamp=datetime.utcnow().isoformat(timespec="seconds"),
            prompt=prompt,
            raw_text=raw_text,
            action=action,
            sanitized_text=sanitized_text,
            detection_summary=detection_summary,
        )
        self.log(record)
