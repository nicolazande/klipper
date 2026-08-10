# Serialservo / TMC4671 axis — design and bring-up reference

Branch `serialservo-tmc4671`, 2026-08-10.  Companion files in this
directory: `tmcl-dst640-reference.ini` (the tuned TMCL-IDE export for
the DMM DST640 that most register defaults derive from) and
`copley-508-notes.md` (Copley 5.08 PVT-timestamp feature notes for the
EtherCAT axes).

Target hardware: DMM DST640 AC servo (pole pairs = 5 per the tuned
config; NOT vendor-documented — verify open loop), 10,000-line ABN
incremental encoder (= 40,000 quadrature counts/rev) plus hall
sensors, TMC4671-LA over SPI on the Main MCU board (STM32H723, USB).

## Architecture

```
klippy (toolhead/trapq)
  -> serialservo_solve.c    samples each constant-accel trapq move
                            every sampling_time; each sample is the
                            state at the END of its interval
  -> serialservo_compress.c converts to wire units in the MCU frame,
                            converts print time to mcu clock, queues
                            "serialservo_queue_step" messages on the
                            stock steppersync/serialqueue path
  -> src/serialservo.c      per interpolation_time tick: evaluates the
                            segment interpolant at the actual transfer
                            time and writes PID_POSITION_TARGET +
                            PID_VELOCITY_OFFSET (feed-forward) via SPI
  -> TMC4671                closes torque/velocity/position loops at
                            25kHz in hardware
```

Chip bring-up/supervision is host-side in `klippy/extras/tmc4671.py`;
the MCU only streams setpoints and reads back actuals.

### Wire protocol

`serialservo_queue_step oid target_position target_velocity clock`
means "be at target_position with target_velocity at clock".  The MCU
interpolates velocity linearly between segment endpoints and position
along the resulting parabola — exact for trapezoidal profiles because
every trapq move is constant-acceleration and the sampler never spans
two moves with one segment.  Between host samples the MCU writes
interpolated setpoints every `interpolation_time` (default 1ms), and
evaluates the interpolant at the actual SPI transfer time (plus
`eval_lead`), which cancels task-scheduling jitter to first order.

Units on the wire (fixed by the chip configuration, selections
velocity/position = 0 with phi_e = ABN):

- position: 1/65536 of an ELECTRICAL revolution, s32 multi-turn.
  pos_reg = mm * pole_pairs * 65536 / rotation_distance
- velocity: ELECTRICAL rpm.  vel_reg = mm/s * 60 * pole_pairs
  / rotation_distance

The electrical (not mechanical) domain is kept deliberately: the tuned
velocity P gain (20000) would overflow its s16 field if rescaled by
pole_pairs for mechanical-domain selections.  All conversions live in
`serialservo_compress.c`/`serialservo.py`; nothing else may convert.

Wire positions are in the MCU frame: `serialservo.py` pushes the
toolhead-frame offset into the compressor
(`serialservo_compress_set_position_offset`) whenever the frame is
rebased, so homing's `set_position(forcepos)` produces a continuous
stream instead of a commanded jump (this replaced the earlier scheme
where a homing rebase slammed the axis by 1.5x the axis length).

### MCU behavior (src/serialservo.c)

- A pacing timer wakes a task; ALL SPI happens in task context (a
  timer callback doing 130-170us of SPI with interrupts masked was
  silently dropping RS485 rx bytes on the old code; USB now, same
  rule).
- Empty queue = hold last target, never extrapolate.  A move arriving
  with its clock already in the past shuts down (host scheduling
  fault), except while the reset flag is set after a homing stop, when
  in-flight moves are silently discarded (same as stock steppers).
- Endstop trigger (trsync, IRQ context): timer deleted, queue drained,
  stop deferred to the task, which reads PID_POSITION_ACTUAL and
  writes it back as the target — hold in place.  Never "position 0"
  (the old code commanded a full-torque slew to encoder zero).
- MCU shutdown: de-energize is done by `config_spi_shutdown` canned
  messages registered by tmc4671.py (motion mode 0, feed-forward 0,
  UQ_UD 0, power stage off), replayed by spidev_shutdown with all CS
  raised first.  serialservo's own shutdown handler touches no SPI.
- Following-error supervision: every `ferror_ticks` the task compares
  the last written target against PID_POSITION_ACTUAL and shuts down
  beyond `ferror_window` (config `following_error`, default 2mm).
  The -LA silicon has NO watchdog and never faults autonomously; this
  window plus the fault polling in tmc4671.py is the protection.

## tmc4671.py — chip lifecycle

Connect (nothing can move yet):
1. Safe state: power stage off (PWM_CHOP=0), motion mode 0, all
   targets/openloop/UQ_UD/PHI_E_EXT zeroed, STATUS_FLAGS cleared.
2. Static configuration written with readback-verify (5 retries; the
   chip has no SPI CRC).  SPI runs at 1MHz: the chip erratum documents
   MSB read corruption at higher pauseless-read rates.
3. ADC offset calibration: zero-current average of ADC_RAW (bank 0),
   sanity-checked to 25-75%% of full scale.  Runs EVERY boot; offsets
   are never transplanted (the three offset pairs floating around the
   old code/dump/logs are all per-board per-boot values).
4. Encoder alignment per `align_mode`:
   - `forced` (default): park rotor at phi_e=0 with UD-only voltage
     ramp (`align_voltage`, `align_delay`), zero the decoder count.
     MOVES THE MOTOR up to half an electrical rev (36 deg mech = up to
     ~1mm of Z at 10mm lead).  Fine for bring-up; power stage is
     switched off again right after.
   - `hall`: copy the hall electrical angle into the ABN phi_e offset;
     zero motion, +-30 deg electrical accuracy, requires commissioned
     hall polarity/direction (see bring-up).
   - `manual`: trust `driver_ABN_DECODER_PHI_E_PHI_M_OFFSET` etc. from
     the config (for N-channel/cln schemes later).
5. SPI handoff to the MCU streamer, host frame anchored from
   PID_POSITION_ACTUAL.

Enable (stepper_enable on): power stage on, position target seeded by
writing PID_POSITION_ACTUAL (the documented auto-copy into
PID_POSITION_TARGET makes loop closure jump-free), motion mode 3.
Fast — no calibration at enable time, so it cannot race move flushing.
Disable / M84: motion mode 0, feed-forward zeroed, power stage off
(freewheel).  Motion after a disable requires re-homing (motor_off
clears kinematics limits), which re-anchors the frame.

Supervision (while enabled): 1Hz STATUS_FLAGS poll — three strikes on
adc_i_clipped(26) / aenc_clipped(27) / not_PLL_locked(19) shuts down —
plus round-robin scrubbing of every static register against its
intended value.

Commands: `INIT_TMC4671`, `SET_TMC4671_FIELD`, `SET_TMC4671_CURRENT`,
`DUMP_TMC4671`, `TMC4671_CALIBRATE_ADC`, `TMC4671_ALIGN_ENCODER
[MODE=forced|hall]`, `TMC4671_STATUS`, `TMC4671_MONITOR [PERIOD=]
[COUNT=] [ENABLE=0]` (live encoder count / abn+hall phi_e / target /
actual / velocity to console and log — the bring-up instrument).

## Configuration reference

```
[mcu servo]
serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_...   # Main MCU board (USB)
restart_method: command

[serialservo_z]
mcu: servo                  # which mcu runs the servo (default "mcu")
rotation_distance: 10.0     # mm of travel per motor revolution (REQUIRED)
pole_pairs: 5               # must match [tmc4671] (REQUIRED)
sampling_time: 0.01         # host segment period (s)
interpolation_time: 0.001   # mcu setpoint update period (s)
following_error: 2.0        # shutdown window in mm (0 disables)
endstop_pin: ^servo:PC14
position_min: -2000
position_endstop: 0.5
position_max: 2000
homing_speed: 5.

[tmc4671 serialservo_z]
cs_pin: servo:PB4
spi_bus: spi2               # spi_speed defaults to 1MHz - keep it
run_current: 8.4            # amps - ONLY meaningful once
current_scale_ma_per_lsb: 1.0   # ...this is measured for OUR board
pole_pairs: 5
encoder_resolution: 40000   # 4x line count (10,000 lines)
encoder_direction: True     # tuned dump had the direction bit set
velocity_limit: 3000        # mechanical rpm
dead_time_ns: 250           # MUST match OUR gate driver, scope-verify
align_mode: forced
align_voltage: 1000
align_delay: 1.0
#adc_i_select: 0x18000100   # board phase/shunt routing (demo value)
#analog_input_stage_cfg: 0x00044400
```

Every chip field also accepts `driver_<FIELDNAME>:` overrides (PID
gains ship with the tuned DST640 values).

## Values taken from the tuned TMCL-IDE export

Motor-bound, transplanted: pole_pairs=5, PPR=40000, decoder direction
bit, PID gains (torque/flux 0x00F5/0x00C1, velocity 0x4E20/0x04B0,
position 0x0050/0x0014), PWM 25kHz, SVPWM on, PIDOUT_UQ_UD limit
0x5A81, dsADC MDEC 334/334, MCLK_B=0.
NOT transplanted (power-stage-bound or runtime state): ADC offsets
(calibrated per boot), ADC_I_SELECT top byte (board routing), dead
time (0x1919 was the eval stage), decoder count/count_n (snapshots),
phi_e offset -63 (wizard residue; alignment re-derives it every boot),
torque limit 6000 (recompute in OUR mA/LSB), velocity limit 4000
(recompute from axis kinematics), config-RAM writes (all zeros =
silicon defaults; the tune uses no biquad filters).

## Bring-up procedure (new Main MCU board)

1. Flash Katapult (SWD), then klipper via katapult.  Config as above,
   motor DECOUPLED from the axis for first power-on.
2. `DUMP_TMC4671 STEPPER=serialservo_z REGISTER=CHIPINFO_DATA` —
   expect 0x34363731 ("4671"); confirms SPI wiring at 1MHz.
3. `TMC4671_CALIBRATE_ADC` — offsets should land near 0x8000 and
   repeat within a few LSB.  Far-off values = check shunt/amp wiring
   and `adc_i_select` / `analog_input_stage_cfg` for our schematic.
4. Open-loop commissioning (motor decoupled):
   `SET_TMC4671_FIELD FIELD=... ` per the datasheet ch.12 procedure or
   temporarily `align_mode: forced` + enable, watching
   `TMC4671_MONITOR`: abn_count must increase for positive phi_e and
   cover exactly 40000 counts per 5 electrical revs (verifies
   pole_pairs=5, encoder_direction, phase order).  A swapped phase
   pair flips direction — fix wiring, not software.
5. Hall commissioning: sweep PHI_E_EXT slowly, record hall_phi_e vs
   phi_e from `TMC4671_MONITOR`; derive HALL_MODE polarity/direction
   (+ position registers if edges deviate from 60 deg).  Then
   `align_mode: hall` gives zero-motion connects.
6. Close the loop: enable (G28 Z homes and anchors the frame), small
   moves, watch following error via `TMC4671_STATUS`.
7. Load tuning: PID gains are the demo-board tune; re-verify on the
   real axis inertia.  Torque/flux gains scale with (demo A/LSB)/(our
   A/LSB) as a starting point.

## Open hardware questions (answers change config values)

- Motor variant from the nameplate (60V "T" class assumed: 8.4A cont /
  21A peak, Kt 0.181 Nm/A).  All current limits depend on it.
- Shunt value + current-amp gain of OUR power stage ->
  `current_scale_ma_per_lsb`.  Until measured, `run_current` amperes
  are nominal, not calibrated.
- Gate driver + FET switching times -> `dead_time_ns` (scope-verify;
  250ns is the eval-board value, the old hardcoded 2.55us wasted 12.75%
  of the PWM period).
- Silicon must be TMC4671-LA (not -ES): SVPWM benefit, watchdog
  removal, SPI read behavior all assume -LA.
- DST640 star point must be isolated for SVPWM (normal for AC servos).
- Encoder electrical interface: 10k-line + hall is not a DMM catalog
  fit — check differential (RS422, needs line receivers) vs
  single-ended, supply voltage.
- Brake output (bench config had [output_pin brake_enable] with
  shutdown_value 0 = brake engages on shutdown - keep that pattern on
  the new board) and regen: ADC_VM_LIMITS is unconfigured; the chip
  has no overvoltage shutdown, so decel from high rpm pumps the DC
  link.  Decide brake-chopper hardware vs software decel limits.

## Known limitations / deferred items

- Daisy-chained TMC4671s are not supported (single chip per SPI CS).
- motion_report is not wired for servo axes (its dump format is
  step-based); use TMC4671_MONITOR / TMC4671_STATUS instead.  A
  sensor_bulk-based streaming path is the natural upgrade.
- FORCE_MOVE/STEPPER_BUZZ raise a clean error on servo axes (the stock
  helper swaps in a step-based kinematics object that does not match
  the servo solver ABI).
- Dual-carriage support in kinematics/hash.py still has the pre-rework
  wiring and is untested.
- stm32h7_spi.c retains a board-specific pin map (works for this
  board; should become Kconfig pins eventually).
- klippy still hard-links the EtherCAT master library at chelper build
  time; hosts without it cannot start klippy at all.
