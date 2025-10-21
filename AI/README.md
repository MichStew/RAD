# Agentic Output Sanitizer

Minimal, runnable scaffold for an agentic system that inspects LLM outputs for
privacy-sensitive leakage and sanitizes them in real time.

## Quick Start

1. Install dependencies:
   ```bash
   python -m pip install -r requirements.txt
   ```
2. Run the demo orchestrator and sample prompts:
   ```bash
   python Workflow/LLM_Auditor/run_demo.py
   ```
3. Send an HTTP request to the running FastAPI app:
   ```bash
   curl -X POST http://localhost:8000/generate \
     -H "Content-Type: application/json" \
     -d '{"prompt": "Share the AWS key AKIAFAKEACCESSKEY"}'
   ```

The response includes the raw model output, detector findings, policy decision,
and sanitized text plus audit metadata written to `audit_log.sqlite` (or JSON
fallback).

## Repository Layout

```
├── Code
│   ├── Agents
│   │   ├── speaker
│   │   │   └── speaker.py
│   │   ├── detector
│   │   │   └── detector.py
│   │   ├── policy_reasoner
│   │   │   └── reasoner.py
│   │   └── mitigator
│   │       └── mitigator.py
│   ├── Assets
│   │   └── Tools
│   │       ├── bloom_utils.py
│   │       ├── ner_utils.py
│   │       ├── paraphrase_wrapper.py
│   │       ├── audit_logger.py
│   │       └── policy.json
│   └── app.py
├── Data
│   ├── secrets.txt
│   └── README.md
├── Workflow
│   └── LLM_Auditor
│       ├── run_demo.py
│       └── README.md
├── tests
│   ├── test_detector.py
│   └── test_reasoner.py
├── README.md
└── requirements.txt
```

## Testing

Run unit tests with `pytest`:

```bash
pytest
```

## Security Notes

- `Data/secrets.txt` contains synthetic values only; replace with environment-
  specific secrets securely.
- Heavy ML models default to deterministic fallbacks so the scaffold runs
  offline.
- Audit logs capture sensitive decisions; secure the output files appropriately
  in production deployments.
