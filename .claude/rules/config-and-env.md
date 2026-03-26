---
paths:
  - "config/**/*"
  - "src/**/config/**/*"
  - "**/*mower_config*"
---

# Config and env

- Treat `config/mower_config.schema.json` as authoritative unless repo evidence shows a different source of truth.
- Keep the deprecated shell example aligned while it still exists.
- Remember that `src/open_mower/config/mower_config.sh.example` is only a redirect stub.
- Document new env vars, config names, and loading behavior when they change.
- Do not let config drift widen silently.
