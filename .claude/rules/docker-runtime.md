---
paths:
  - "docker/**/*"
  - "devenv/**/*"
  - ".devcontainer/**/*"
---

# Docker and runtime

- Distinguish runtime-image behavior from development-container behavior.
- Preserve the default versus legacy runtime split unless the task explicitly changes it.
- Treat entrypoint edits as high-risk because they change environment loading and startup assumptions.
- Update the Docker docs when runtime assumptions change.
