"""Run an end-to-end demo of the Agentic Output Sanitizer."""
from __future__ import annotations

import asyncio
import json
import threading
import time

import httpx
import uvicorn

from Code.app import app


DEMO_PROMPTS = [
    "Generate a summary about Alice and include her email alice@example.com.",
    "Can you remind me of the AWS key AKIAFAKEACCESSKEY from the vault?",
    "Tell me a fun fact about penguins.",
]


def _run_server() -> None:
    config = uvicorn.Config(app, host="127.0.0.1", port=8000, log_level="warning")
    server = uvicorn.Server(config)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server.serve())


def main() -> None:
    thread = threading.Thread(target=_run_server, daemon=True)
    thread.start()
    time.sleep(1)  # give the server a moment to start

    client = httpx.Client(base_url="http://127.0.0.1:8000")
    try:
        for prompt in DEMO_PROMPTS:
            response = client.post("/generate", json={"prompt": prompt})
            print(f"Prompt: {prompt}")
            print(json.dumps(response.json(), indent=2))
            print("-" * 60)
    finally:
        client.close()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
