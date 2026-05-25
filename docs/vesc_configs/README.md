# VESC config snapshots

Purpose: keep a repo-tracked copy of the live `Mowrator` VESC configuration so future agents and contributors can see the last known controller state without querying the mower first.

## What belongs here

This directory stores XML dumps read from the live mower with the official VESC Tool CLI:

- `left-drive-mcconf.xml`
- `left-drive-appconf.xml`
- `right-drive-mcconf.xml`
- `right-drive-appconf.xml`
- `mower-mcconf.xml`
- `mower-appconf.xml`

These files are snapshots, not the source of truth for writing config. The live VESCs remain the runtime source of truth.

The snapshots can also be firmware-layout specific. If an ESC is reflashed and comes back with a different firmware family or XML shape, do not blindly push an older snapshot back onto it. Read back fresh XML from the reflashed controller first, patch the mower-specific values onto that new base, write it, and then refresh this directory with the final readback.

## Update rule

If you change any live VESC setting on the mower, update this directory in the same change.

At minimum:

1. Stop the mower runtime on the Pi.
2. Re-dump all VESC configs from the Pi:

```bash
~/open_mower_ros/utils/scripts/vesc/dump_all_vesc_configs.sh ~/vesc-config-dumps/current
```

3. Copy the fresh XMLs into this directory:

```bash
rsync -av \
  mowrator@mowrator.local:/home/mowrator/vesc-config-dumps/current/ \
  /Users/martinhaghani/Code/open_mower_ros/docs/vesc_configs/
```

4. Update [../VESC_MAINTENANCE.md](../VESC_MAINTENANCE.md) if the maintenance procedure changed.

Do not leave live VESC changes undocumented.
