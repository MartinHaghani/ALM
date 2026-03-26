# Config guide

Purpose: guidance for editing config artifacts under `config/`.

## Source of truth

- `mower_config.schema.json` is the authoritative structured config artifact.
- `mower_config.sh.example` is Deprecated but still maintained for compatibility.
- `../src/open_mower/config/mower_config.sh.example` is only a redirect stub.

## Working rules

- Keep the schema, deprecated shell example, and docs aligned when config semantics change.
- Do not hide observed drift. Current examples include `ESC_TYPE` versus `OM_MOWER_ESC_TYPE` and the `Sabo` hardware preset gap.
- If launch-time loading behavior changes, also update [../docs/CONFIGURATION.md](../docs/CONFIGURATION.md).

## Required companion docs

- [../docs/CONFIGURATION.md](../docs/CONFIGURATION.md)
- [../docs/BUILD_AND_RUN.md](../docs/BUILD_AND_RUN.md)
