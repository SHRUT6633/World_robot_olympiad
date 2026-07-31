<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/datasheets/lipo_battery.md
Component: LiPo 2S battery (power source)
=============================================================================
-->

# LiPo 2S Battery

The single power source. Everything draws from it through the
protection chain (switch → fuse → polarity protection → cutoff).

## Specs

| Parameter | Value |
|-----------|-------|
| Chemistry | LiPo 2S (2 cells in series) |
| Nominal voltage | 7.4V (3.7V per cell) |
| Capacity | 2000 mAh |
| Energy | 14.8 Wh |
| C-rating | 25C → ~50 A burst (far above our ~5 A peak) |
| Cutoff voltage | 3.3V/cell → 6.6V total (buzzer warns) |
| Weight | ~100 g (fits the 1.5 kg limit) |
| Connector | XT60 |

## Power rail connection

- Battery → main switch → 10A fuse → polarity protection → three rails
  (motor / 5V / servo). See POWER_DISTRIBUTION.md Section 2.

## Protection notes

- Below 3.3V/cell the LiPo is permanently damaged — the low-voltage
  cutoff + buzzer is mandatory, not optional.
- Runtime estimate: ~35–40 min of mixed driving at ~18.2W average
  load; a 3-minute WRO round uses less than 10%.
- Store at ~3.8V/cell; charge only with a LiPo balance charger.

## Official datasheet

- None — generic 2S LiPo. Use the manufacturer's charge/discharge
  specifications for the exact pack.
