# EtherCAT subsystem — changes made during board bring-up (2026-08-07)

Handoff notes for the firmware engineer. Context: during bench testing of
the RS485 board periphery (fans, thermistors), every klipper restart took
~100 seconds and the process segfaulted. This document lists exactly what
was changed in the EtherCAT area, why, what was deliberately **not**
changed, and things found in passing that may be useful.

## Symptoms that motivated the changes

1. **FIRMWARE_RESTART took ~100-108s and crashed the process.** Measured
   breakdown: ~3s until SIGSEGV in the EtherCAT reconnect, +10s systemd
   `RestartSec`, +~90s systemd waiting on a missing `can3` CAN device
   (unrelated to EtherCAT — see "System-level" below).
2. The SIGSEGV: `ecrt_request_master(0)` returned NULL on the second
   in-process reservation (kernel: "Master already in use!") because the
   first session's master was never released; the NULL was then
   dereferenced by `ecrt_master_slave_config()`.

## Code changes (commit `35ee2f72`)

### `klippy/chelper/ethercatqueue.c`
- **`ethercatqueue_init`**: NULL-check after `ecrt_request_master(0)` →
  `ret = -1; goto fail;`. Note this check existed in the pre-squash
  history (`d8c1389f`, `HANDLE_ERROR(!master->master, fail)`) and was
  lost in the `e6d32ff3` squash — this restores it.
- **`ethercatqueue_exit`**: after the cyclic thread is joined, call
  `ecrt_release_master()` (which deactivates internally per ecrt.h) and
  clear every handle derived from the master: `master->master`,
  `domains[i].domain/domain_pd`, `monitor[i].slave` and the four
  `*_sdo` pointers. Without the release, the kernel keeps the master
  reserved by the PID and any same-process reconnect gets EBUSY. Clearing
  `monitor[i].slave` also matters for the re-init path: init's
  "skip already configured slaves" compares against it, and a stale
  pointer could alias a new allocation.

### `klippy/ethercathdl.py`
- `_start_session` now honors the `ethercatqueue_init` return code
  (the FFI already declared `int`, and the C fail path already reported
  the error — the rc was just discarded): on failure, no threads are
  started and the session returns False, so `connect_ethercat`'s
  existing 90s retry/timeout handles it.
- 1s pause between connect retries (same idiom as the serial connect
  loop) instead of a tight loop.

### `klippy/mcu.py`
- New `[mcu]` option **`enable_ethercat`** (default **True** = behavior
  unchanged). With `False`: no `EthercatReader` is constructed, no
  master reservation, no cyclic thread, and the `drivesync_alloc` /
  `drivesync_flush` / `drivesync_set_time` call sites are guarded
  (`self._drivesync = None`). `clocksync.connect(serial, None)` was
  already tolerated (its only consumer checks `is not None`), and
  fileoutput mode already ran this way.
- `_mcu_identify` catches `ethercathdl.error` alongside `serialhdl.error`
  (per the existing TODOs at mcu.py "extend to ethercat").

### `klippy/ethercatservo.py`
- Four early-return guards for the disabled state
  (`self._mcu._ethercat is None`): `PVT_endstop._build_config`,
  `EthercatServo._build_config`, `note_homing_end`,
  `_query_mcu_position`. Purpose: `[ethercatservo_x/y]` +
  `kinematics: hash` can stay in printer.cfg during bench testing.
  **Important hazard these guards close**: `MCU.lookup_command(...,
  serial=None)` falls back to the *serial* MCU dictionary, and
  `src/stepper.c` defines commands with identical formats
  (`reset_step_clock`, `stepper_get_position`, ...) — without the
  guards, drive commands could silently bind to the RS485 MCU. With the
  guards, homing/moving the disabled rails raises a clean Python error.

### `src/serialservo.c` (TMC4671-over-SPI area) — commit `b893f9de`
- `serialservo_stop()` skips `tmc_set_position(d, 0)` when `d->spi` is
  NULL (i.e. `config_serialservo_spi` was never sent because the
  `[tmc4671 ...]` section is commented out). Previously this ran during
  MCU shutdown handling, dereferenced the NULL spidev, and the nested
  shutdown wedged the chip until the watchdog reset it — this was the
  root cause of the "MCU spontaneously restarts" problem that broke the
  RS485 bring-up. See `docs/RS485-Status.md` for the full chain.
- Related, `src/sched.c`: `run_shutdown()` now runs the DECL_SHUTDOWN
  handler list at most once per shutdown, so a handler that itself
  raises `shutdown()` can no longer re-enter the list forever with
  interrupts disabled.

## System-level change (not in the repo)

- `/etc/systemd/system/klipper.service.d/socketcan.conf`: removed
  `After=socketcan@can3.service` (kept `Wants=`). With the candleLight
  CAN adapter unplugged, the ordering made every klipper start wait
  systemd's 90s device timeout for the missing `can3` interface. With
  `Wants=` alone, `socketcan@can3` is still pulled up (and configures
  the interface exactly as before) whenever the adapter is present —
  klipper just no longer blocks on it. Original file preserved as
  `socketcan.conf.bak`.

## Verification

- FIRMWARE_RESTART (EtherCAT **enabled**): 108s → **3.2s**, repeatable,
  zero SEGV; kernel log shows clean release → re-request → domain WC 3/3
  → slaves reaching OP after the in-process reconnect.
- FIRMWARE_RESTART (EtherCAT **disabled**): 3.2s; full printer.cfg
  (hash kinematics + both ethercatservo sections) loads to "ready";
  RS485 periphery fully functional; the (by-design) busy-spin cyclic
  core is not started, freeing one of the Pi's 3 cores.

## Deliberately NOT changed

- The **busy-spin cyclic reactor** (`PR_OFFSET = INT32_MAX` disabling
  poll(), commits "spiining reactor" / "increase reactor resolution") —
  recognized as an intentional latency design; it pegs one core for the
  session. If ever revisited: a small PR_OFFSET (1-2ms) would re-enable
  poll() sleep for most of each 10ms cycle.
- `CHECK_MASTER_STATE = 0` and the absence of slave-presence checks
  (zero powered slaves currently connect "successfully" and the DS402
  state machine runs against an all-zero status word).
- The existing TODOs left in place: `ethercatqueue.c` "add proper error
  handling" (master/domain state), `mcu.py` "TODO: add drivesync check"
  in `check_active`, "TODO: add ethercat counterpart" for the shutdown
  command, `ethercatservo.py` timing/error handling TODOs.

## Found in passing (no action taken)

- **`ethercatqueue_free()` latent bug**: it ends with `free(sq)`, but
  since commit `3faaa4d4` the queue is the static `ethercatdata` —
  wiring this function up as-is would corrupt the heap. It currently has
  no callers. Drop the `free(sq)` line if it is ever used.
- `pollreactor_alloc` in init leaks the previous session's pollreactor
  on reconnect (small, bounded by restart count).
- `EthercatReader.stats()` exists but has no callers.
- The per-session Python bg thread and C cyclic thread are correctly
  joined on disconnect (that part of the lifecycle was already sound).
