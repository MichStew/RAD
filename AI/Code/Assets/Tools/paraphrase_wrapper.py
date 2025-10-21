"""Lightweight paraphrasing helper with optional transformers backend."""
from __future__ import annotations

import logging

try:  # pragma: no cover - optional dependency
    from transformers import AutoModelForSeq2SeqLM, AutoTokenizer
    import torch
except Exception:  # pragma: no cover - degrade gracefully
    AutoModelForSeq2SeqLM = None  # type: ignore
    AutoTokenizer = None  # type: ignore
    torch = None  # type: ignore

LOGGER = logging.getLogger(__name__)


class Paraphraser:
    """Generate a paraphrased version of text with safe fallbacks."""

    def __init__(self, model_name: str = "t5-small") -> None:
        self.model_name = model_name
        self._model = None
        self._tokenizer = None
        self._dummy_mode = True
        if AutoModelForSeq2SeqLM and AutoTokenizer:
            try:
                self._tokenizer = AutoTokenizer.from_pretrained(model_name)
                self._model = AutoModelForSeq2SeqLM.from_pretrained(model_name)
                if torch is not None:
                    self._model.eval()
                self._dummy_mode = False
                LOGGER.info("Loaded paraphraser model %s", model_name)
            except Exception as exc:  # pragma: no cover - degrade gracefully
                LOGGER.warning("Paraphraser fallback: %s", exc)
                self._dummy_mode = True
        else:
            LOGGER.warning("Transformers unavailable; paraphraser will use dummy mode")

    def paraphrase(self, text: str) -> str:
        if self._dummy_mode or not self._model or not self._tokenizer:
            return f"[Paraphrased] {text}"

        input_text = f"paraphrase: {text}"
        inputs = self._tokenizer([input_text], return_tensors="pt", truncation=True)
        if torch is not None:
            inputs = {k: v.to(self._model.device) for k, v in inputs.items()}
        with torch.no_grad():  # type: ignore[attr-defined]
            outputs = self._model.generate(**inputs, max_length=128)
        return self._tokenizer.decode(outputs[0], skip_special_tokens=True)
