#!/usr/bin/env python3
"""Deny git push to origin (JonathanV0/6925-Rebuilt). Allow personal."""

from __future__ import annotations

import json
import sys

FORBIDDEN_REMOTES = {"origin"}
FORBIDDEN_URL_SNIPPETS = ("jonathanv0/6925-rebuilt",)


def payload() -> dict:
    raw = sys.stdin.read()
    if not raw.strip():
        return {}
    try:
        data = json.loads(raw)
        return data if isinstance(data, dict) else {}
    except json.JSONDecodeError:
        return {}


def remote_after_push(command: str) -> str | None:
    tokens = command.replace("\n", " ").split()
    try:
        push_at = next(i for i, t in enumerate(tokens) if t == "push")
    except StopIteration:
        return None
    if push_at == 0 or "git" not in tokens[push_at - 1]:
        # still treat a bare "push origin" after git -C etc.
        pass
    for token in tokens[push_at + 1 :]:
        if token.startswith("-"):
            continue
        return token.lower()
    return None


def is_forbidden(command: str) -> bool:
    lowered = command.lower()
    if "git" not in lowered or "push" not in lowered:
        return False
    if any(snippet in lowered for snippet in FORBIDDEN_URL_SNIPPETS):
        return True
    remote = remote_after_push(command)
    return remote in FORBIDDEN_REMOTES


def main() -> int:
    command = str(payload().get("command") or "")
    if is_forbidden(command):
        json.dump(
            {
                "permission": "deny",
                "user_message": "Blocked push to origin (JonathanV0/6925-Rebuilt). Push personal/n4 only.",
                "agent_message": "Do not git push origin. Use: git push personal n4",
            },
            sys.stdout,
        )
        return 0
    json.dump({"permission": "allow"}, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
