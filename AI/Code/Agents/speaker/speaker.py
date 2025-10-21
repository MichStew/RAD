"""Speaker agent that wraps a lightweight local language model."""
from __future__ import annotations

import logging
from typing import Any, Dict, List

try:
    from transformers import AutoModelForCausalLM, AutoTokenizer
    import torch
except Exception:  # pragma: no cover - optional dependency
    AutoModelForCausalLM = None  # type: ignore
    AutoTokenizer = None  # type: ignore
    torch = None  # type: ignore


LOGGER = logging.getLogger(__name__)


class LocalSpeaker:
    """Generate candidate text from a prompt using a local LLM.

    The class attempts to load a small HuggingFace causal language model. If the
    dependency or weights are not available, it falls back to a deterministic
    dummy generator that returns a canned response. The return value always
    includes text, per-token log probabilities, and model metadata to facilitate
    downstream exposure heuristics.
    """

    def __init__(self, model_name: str = "sshleifer/tiny-gpt2", max_new_tokens: int = 50) -> None:
        self.model_name = model_name
        self.max_new_tokens = max_new_tokens
        self._model = None
        self._tokenizer = None
        self._dummy_mode = True
        if AutoModelForCausalLM and AutoTokenizer:
            try:
                self._tokenizer = AutoTokenizer.from_pretrained(model_name)
                self._model = AutoModelForCausalLM.from_pretrained(model_name)
                if torch is not None:
                    self._model.eval()
                self._dummy_mode = False
                LOGGER.info("Loaded HuggingFace model %s", model_name)
            except Exception as exc:  # pragma: no cover - handled gracefully
                LOGGER.warning("Falling back to dummy speaker due to: %s", exc)
                self._dummy_mode = True
        else:
            LOGGER.warning("Transformers not available, using dummy speaker")
            self._dummy_mode = True

    def generate(self, prompt: str, **generation_kwargs: Any) -> Dict[str, Any]:
        """Generate text for a prompt.

        Args:
            prompt: The text prompt provided by the user.
            **generation_kwargs: Optional keyword arguments forwarded to the
                HuggingFace ``generate`` method when available.

        Returns:
            Mapping containing ``text``, ``logprobs`` (per-token list), and
            ``meta`` data about the generation.
        """

        if self._dummy_mode or not self._model or not self._tokenizer:
            text = f"Dummy response to: {prompt.strip()}"
            tokens = text.split()
            logprobs = [-0.5 for _ in tokens]
            meta = {
                "model": "dummy",
                "num_tokens": len(tokens),
            }
            return {"text": text, "logprobs": logprobs, "meta": meta}

        kwargs = {
            "max_new_tokens": self.max_new_tokens,
            "do_sample": False,
            "return_dict_in_generate": True,
            "output_scores": True,
        }
        kwargs.update(generation_kwargs)

        input_ids = self._tokenizer(prompt, return_tensors="pt").input_ids
        if torch is not None:
            input_ids = input_ids.to(self._model.device)

        with torch.no_grad():  # type: ignore[attr-defined]
            outputs = self._model.generate(input_ids, **kwargs)

        sequence = outputs.sequences[0]
        generated_ids = sequence[input_ids.shape[-1] :]
        text = self._tokenizer.decode(generated_ids, skip_special_tokens=True)

        logprobs: List[float] = []
        if outputs.scores:
            for score_tensor, token_id in zip(outputs.scores, generated_ids):
                prob = score_tensor.softmax(-1)[0, token_id]
                logprobs.append(float(torch.log(prob)))  # type: ignore[arg-type]
        else:  # pragma: no cover - extremely small models may not expose scores
            logprobs = [-1.0] * len(generated_ids)

        meta = {
            "model": self.model_name,
            "num_tokens": len(generated_ids),
        }
        return {"text": text, "logprobs": logprobs, "meta": meta}
