#!/usr/bin/env python3
"""Strip machine-specific PROS template cache paths out of project.pros.

`pros conduct` records the absolute local filesystem path each template was
fetched from (e.g. `C:\\Users\\yourname\\AppData\\Roaming\\PROS\\templates\\...`
on Windows, `/Users/yourname/Library/Application Support/PROS/templates/...`
on macOS). That path leaks your OS username/home layout and is meaningless to
anyone else who clones the repo — their local PROS cache lives elsewhere.

This rewrites every `location` field under `templates.*` to a generic
placeholder, keeping only the `<name>@<version>` suffix PROS actually cares
about for display purposes. It does not affect what `pros conduct
fetch`/`apply`/`upgrade` do locally; those re-resolve real paths on demand.

Run manually:
    python scripts/sanitize_project_pros.py

Or let the `pre-commit` git hook in .githooks/ run it automatically before
every commit (see .githooks/README.md for one-time setup).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PLACEHOLDER_ROOT = "<PROS_TEMPLATES_DIR>"
PROJECT_PROS = Path(__file__).resolve().parent.parent / "project.pros"


def sanitize(data: dict) -> int:
    """Rewrite local `location` fields in-place. Returns the number changed."""
    templates = data.get("py/state", {}).get("templates", {})
    changed = 0
    for template in templates.values():
        location = template.get("location")
        if not isinstance(location, str) or location.startswith(PLACEHOLDER_ROOT):
            continue
        leaf = location.replace("\\", "/").rstrip("/").rsplit("/", 1)[-1]
        template["location"] = f"{PLACEHOLDER_ROOT}/{leaf}"
        changed += 1
    return changed


def main() -> int:
    if not PROJECT_PROS.exists():
        return 0

    data = json.loads(PROJECT_PROS.read_text(encoding="utf-8"))

    changed = sanitize(data)
    if changed == 0:
        print("sanitize_project_pros: already clean, nothing to do.")
        return 0

    PROJECT_PROS.write_text(json.dumps(data, indent=4) + "\n", encoding="utf-8")
    print(f"sanitize_project_pros: scrubbed {changed} template location path(s) in project.pros.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
