"""FastAPI orchestrator for the Agentic Output Sanitizer."""
from __future__ import annotations

from fastapi import FastAPI
from pydantic import BaseModel

from Code.Agents.speaker import LocalSpeaker
from Code.Agents.detector import EnsembleDetector
from Code.Agents.policy_reasoner import PolicyReasoner
from Code.Agents.mitigator import Mitigator
from Code.Assets.Tools.audit_logger import AuditLogger

app = FastAPI(title="Agentic Output Sanitizer")

speaker = LocalSpeaker()
detector = EnsembleDetector()
reasoner = PolicyReasoner()
mitigator = Mitigator()
audit_logger = AuditLogger()


class PromptRequest(BaseModel):
    prompt: str


@app.post("/generate")
def generate(request: PromptRequest):
    generated = speaker.generate(request.prompt)
    text = generated.get("text", "")
    logprobs = generated.get("logprobs", [])
    detection = detector.analyze(text, {"logprobs": logprobs})
    decision = reasoner.decide(detection)
    mitigation = mitigator.apply(text, decision["action"], detection)

    audit_logger.log_decision(
        prompt=request.prompt,
        raw_text=text,
        action=decision["action"],
        sanitized_text=mitigation["sanitized_text"],
        detection_summary=detection,
    )

    return {
        "prompt": request.prompt,
        "raw_text": text,
        "decision": decision,
        "sanitized_text": mitigation["sanitized_text"],
        "details": mitigation["details"],
    }
