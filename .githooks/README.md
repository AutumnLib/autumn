# Git hooks

Tracked hooks live here instead of `.git/hooks/` (which git never versions).
Enable them once per clone:

```bash
git config core.hooksPath .githooks
```

## pre-commit

Runs `scripts/sanitize_project_pros.py` and re-stages `project.pros` so no
commit ever ships your local PROS template cache path (leaks OS
username/home directory, meaningless on anyone else's machine). Safe to run
standalone too:

```bash
python scripts/sanitize_project_pros.py
```
