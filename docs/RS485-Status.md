# RS485 Half-Duplex Bring-Up — Status & Notes

Working notes for the `rs485-experimental` branch. This documents the root
causes found on 2026-08-07, the fixes applied, and every temporary
diagnostic that must be removed/reverted once the link is verified stable.

## Setup

- MCU: STM32H723 @ 400MHz, USART2 on PD6 (RX) / PD5 (TX), hardware
  driver-enable on PD4 (USART CR3 DEM), 250000 baud.
- Host: Raspberry Pi + FTDI USB-RS485 cable (`/dev/ttyUSB0`, FT232R).
  The cable handles its own direction (TXDEN, released at the stop-bit
  edge) and ships with local echo **disabled** (CBUS4=TXDEN gates the
  transceiver's RE#) — verified empirically on this unit: no echo.
- Bootloader: Katapult with RS485 support, built on the Mac
  (`v0.0.1-117-gb08527e`). **That source exists only on the Mac — push it
  to `Warmbird/katapult`.**

## Root causes of the "works a few seconds then silently stalls" symptom

Established from a captured wire trace
(`/home/ubuntu/printer_data/logs/mcu-rs485-wire.trace`, session of
2026-08-07 ~05:46 UTC) plus klippy.log forensics of all Aug 5 sessions:

1. **The MCU spontaneously reboots mid-session** (observed three times:
   ~31s, ~5s, ~28s after connect; cause not yet identified — the new
   `get_reset_reason` diagnostic will discriminate watchdog vs pin vs
   brownout on the next occurrence). On reboot the MCU's sequence counter
   resets to 0x10 and its config is lost.
2. **The half-duplex host could not recover from (or even detect) the
   reboot**: with one un-ACKed block outstanding the send window freezes,
   and the rebooted MCU's NAKs (and its queued `starting` announcement)
   computed to `rseq > send_seq`, so the host discarded every post-reboot
   frame and retransmitted the same block with backoff to 5s forever.
   Mainline full-duplex escapes this because new blocks keep advancing
   `send_seq`; the half-duplex window cannot. **Fixed** (see below).
3. **The stall was silent** because `[statistics] active=0` at commit
   be5d3bd7 skipped all stats callbacks, and `toolhead.stats()` is the only
   caller of the MCU timeout check — no timeout, no log, frozen UI.
   **Fixed** at b2c4f1dd (callbacks always run; `active` only gates log
   output).

The fan stopping and thermistors freezing are what a rebooted
(unconfigured) MCU looks like: all pins return to their power-on state
and no ADC reports are sent.

## Fixes applied on this branch (2026-08-07)

Host (`klippy/chelper/serialqueue.c`):
- Half-duplex peer-reset recovery: CRC-valid frames beyond the send window
  are no longer discarded in half-duplex mode; an empty frame realigns
  `send_seq` (counted as `half_duplex_resyncs` in stats) and data frames
  (e.g. the MCU's `starting`) are delivered, so klippy now reports
  "MCU 'mcu': spontaneous restart" instead of stalling.
- Retransmit timer is deferred while the MCU's response turn is actively
  arriving (prevents mid-turn retransmit collisions).
- Idle polls are no longer suppressed by a future-scheduled command.
- Short `write()`s are retried so frames cannot be truncated.
- Full-duplex notify semantics restored to mainline (`rseq-2` for data
  frames) for non-RS485 transports.
- `pending_notify` count added to serial stats.
- `serialqueue_alloc_trace` NULL (allocation failure) falls back to
  untraced operation instead of crashing.

Host (Python):
- `serialhdl.py`: FTDI low-latency mode (1ms latency timer) requested on
  half-duplex connects; background-thread exit now fails all pending
  waiters (no more indefinite hangs on link death).
- `mcu.py`: `restart_method: command` waits 100ms (not 15ms) on
  half-duplex before closing the port; logs the MCU's reset reason after
  each connect. `_connect_file` clocksync signature fixed.
- `console.py`: fixed fork clocksync signature crash.

MCU firmware:
- `src/generic/serial_irq.c`: deferred frames are re-stamped with the
  current sequence number at promotion (they previously replayed a stale
  sequence and were discarded by the host — lost one-shot events); stale
  ack/naks are dropped instead of deferred.
- `src/stm32/stm32f0_serial.c`: stale RDR byte flushed at turn end
  (RQR RXFRQ); error flags (PE/FE/NE/ORE) cleared in the ISR; DE
  assertion/deassertion guard time of half a bit programmed (DEAT/DEDT,
  preserved across all CR1 writes); RX drain loop.
- `src/stm32/stm32h7.c`: new `get_reset_reason` command reporting (and
  clearing) RCC->RSR so the next unexplained reboot identifies itself
  (IWDG watchdog / NRST pin / brownout / power-on / software).
- `src/stm32/stm32h7_spi.c`: SPI busy-waits bounded (a hung SPI now
  triggers a clean MCU shutdown instead of spinning with IRQs off until
  the 512ms IWDG hard-resets the chip). Note: `serialservo_event` still
  performs seven blocking SPI transfers + an `output()` from timer-IRQ
  context — acceptable for bring-up, should move to task context before
  production servo use.

Katapult (`~/katapult/scripts/flashtool.py`, local clone):
- On a raw serial device, a failed bootloader CONNECT now automatically
  sends the serial bootloader request magic and retries — so
  `flashtool.py -f` works against a running Klipper without a separate
  `-r` invocation. (Previous "automatic entry doesn't work" was workflow:
  `-f` alone never sent the request, and `-r` was run while the klipper
  service still held the port.)

## TEMPORARY items — remove after RS485 is verified stable

| Item | Where | Action when stable |
| --- | --- | --- |
| `serial_wire_trace:` option in `[mcu]` | `printer.cfg` | Remove the line (13MB RAM + trace file on every timeout/disconnect). The code support can stay. |
| `[statistics] active: 1` | `printer.cfg` | Set back to `0` if log volume is unwanted (safety no longer depends on it). |
| Wire trace file | `/home/ubuntu/printer_data/logs/mcu-rs485-wire.trace` | Delete (appends forever; no rotation). |
| `pollreactor` diagnostic counters | enabled only when tracing | Nothing to do — off when trace is off. |
| `get_reset_reason` + connect log line | firmware + `mcu.py` | Keep (cheap, generally useful) or drop for upstream parity. |

## Known-good flash procedure (from this Pi)

```
sudo service klipper stop
python3 ~/katapult/scripts/flashtool.py \
  -d /dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTBG6Y72-if00-port0 \
  -b 250000 -f ~/klipper/out/klipper.bin
sudo service klipper start
```

Works whether the board is in Katapult or running Klipper (auto-entry).
If the application is hard-wedged, enter Katapult manually (double-tap
reset) and rerun.

## Open questions / next steps

1. **Why does the MCU reboot?** Next stall will log the reset reason at
   reconnect. If `iwdg_watchdog`: something wedges the main loop >450ms —
   instrument further (task breadcrumbs). If `nrst_pin`/`brownout`:
   electrical — look at the AC fan, wiring, PSU.
2. The fork's klippy process segfaults in the EtherCAT chelper on every
   in-process restart when `/dev/EtherCAT0` is missing/busy
   (`Failed to reserve master` → SIGSEGV) — 11 crashes in the Aug 5 logs.
   Unrelated to RS485 but confounds every FIRMWARE_RESTART; worth fixing.
3. One CPU core is pegged at 100% for the life of every klippy session
   (ethercatqueue busy-spin) — predates RS485 work, worth investigating.
4. Push the Mac's katapult RS485 branch (`b08527e`) to `Warmbird/katapult`
   and commit the flashtool auto-entry patch there.
5. `serialservo_event` restructure (see above) before driving Z with it.
