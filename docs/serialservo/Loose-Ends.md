# Open items tracker — serialservo-tmc4671 branch (2026-08-10)

## Blocked on hardware / operator

1. Copley 5.08 firmware not loaded on any of the four AE2-090-14
   drives (0x2014 absent).  Gates the 0x85 timestamp feature (host
   side ready, self-arming).  Pair with assign_activate -> 0x300 in
   canopen/config.json and bench-verify DC convergence.
2. Confirm drive-3 motors hold audibly / move visibly (streaming now
   verified to the drive buffer; commanded motion so far microscopic
   by scaling).
3. Main MCU board: Katapult via SWD first (board untouched until
   then), then a NEW klipper build config: STM32H723, USB on
   PA11/PA12 (current .config is the RS485 support-board build).
4. DST640 nameplate: voltage variant (current limits, Kt) and
   pole_pairs=5 verification (open-loop: 5 electrical revs per mech
   rev).
5. Power-stage values for [tmc4671]: current_scale_ma_per_lsb
   (shunt+amp), dead_time_ns (gate driver, scope-verify),
   adc_i_select / analog_input_stage_cfg (schematic), PA14 enable
   polarity (also SWCLK - driving it blocks SWD until reset),
   TMC4671-LA silicon check, star-point isolation for SVPWM, encoder
   interface (RS422 vs single-ended), brake/regen decision
   (ADC_VM_LIMITS unconfigured; chip has no OV shutdown).
6. Hall commissioning sweep (PHI_E_EXT vs HALL_PHI_E, motor
   decoupled) before align_mode: hall.
7. Push to GitHub (key installed; deferred by request).

## Code triage still open

8. Branch-review findings beyond the two fixed (burst anchor,
   missing return): the full results file has more verified findings
   not yet triaged (session task wdb3024l3 output; includes a
   ferror wrap-suppression remark).
9. TEMPORARY diagnostics to remove after flow verification:
   "ethercat flow"/"ethercat seg" logs in ethercatqueue.c, plus the
   RS485 bring-up diagnostics listed in docs/RS485-Status.md
   (wire trace, statistics active=1).
10. Firmware pin-table gaps for the Main MCU board:
    - I2C4 on PB6/PF15 is NOT in src/stm32/i2c.c (only i2c1-i2c3
      variants exist) - must be added before the I2C periphery works.
    - FDCAN1 PB8/PB9 and FDCAN3 PD12/PD13 Kconfig options exist;
      FDCAN2 PB12/PB13 support unverified; klipper uses one CAN
      interface per mcu regardless.
    - hardware_pwm users must match the stm32 hard_pwm timer table
      (software PWM unaffected); spot-check per fan/heater section.
    - Verified good: all six SPI buses match the pin sheet
      (spi1 PG9/PD7/PG11 ... spi6 PG12/PB5/PC12); all 14 NTC ADC
      pins present in stm32h7_adc.c; USB PA11/PA12 supported; all
      CS/endstop/enable/brake/status pins are plain GPIO.
11. klippy_uds_address lead (odd ethercat-era socket setting) - not
    yet checked in moonraker.conf / launch scripts.
12. Z homing precision: HOMING_SAMPLE_DIST=0.01 sets endstop poll
    rate; verify latency on hardware.
13. serialservo: no daisy-chain support; motion_report not wired
    (sensor_bulk streaming is the upgrade path); stm32h7_spi.c
    board-specific pin map should become Kconfig; hash.py
    dual-carriage path stale/untested.
14. Extruder section rebuild with real pins (bring-up leftover).
15. test/klippy/printers.test broken in the fork; klippy hard-links
    the IgH library at chelper build time (hosts without it cannot
    start klippy).

## EtherCAT decision agenda (with Nicola / Copley)

Full detail in EtherCAT-Sync-Gaps.md.  Headlines: record
format/velocity units + true count scaling (deployed scaling 100 is
dimensionally wrong; linear-scaling work the team already knew
about); throughput ceiling (1 segment/axis/cycle); planner
short-move collapse; end-of-move pose emission + record start/end
semantics (Programmer's Manual now in the shared Drive folder);
error-recovery protocol (CLEAR_ERRORS slot clobber, seq resync,
NOP parking); cyclic-thread pacing; homing frame convention;
FIRMWARE_RESTART stale seq/time_table (one recovery cycle per
restart); 0x2014 into a TxPDO for continuous drift telemetry.
