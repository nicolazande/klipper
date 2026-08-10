# Copley firmware 5.08 — Warmbird application features (from release notes)

Source: user-provided release-note screenshots (3 pages), 2026-08-10.

## Feature 1: Encoder sharing between axes of a multi-axis drive

- New load-encoder type 29: axis uses the encoder physically wired to (and used
  by) another axis of the same multi-axis drive.
- Object 0x2383 sub-index 30 (load encoder type) = 29 selects it. Axis-specific
  objects offset by 0x800 per axis (0x2B83 = axis B load encoder type).
- Load encoder options object 0x2223 (axis A) / 0x2A23 (axis B) encodes:
  bits 0-1 = axis number of the encoder to copy (0=A, 1=B);
  bit 4 = set: copy the other axis' LOAD encoder, clear: copy its MOTOR encoder;
  bit 8 = set: encoder fault on the copied axis propagates to the copying axis;
  other bits reserved.
- Example: axis B copies axis A load encoder -> 0x2B83:30=29, 0x2A23=0x00000010.
- Encoder types can be changed during operation but BOTH axes must be disabled
  while changing the configuration.

## Feature 2: PVT timestamp synchronization (the gap Nicola found)

- Purpose: pin a specific PVT segment to a specific absolute (DC) time; the
  drive then measures queue-pull timing error and compensates.
- Object 0x2010 (8-byte PVT data object; byte 0 = record type). New record:
  byte 0 = 0x85 (timestamp), bytes 1-6 = lower 48 bits of the EtherCAT
  distributed-clock time (nanosecond units) at which the PVT segment sent
  JUST PRIOR should be processed; byte 7 unused.
- On pulling the timestamp from its buffer, the drive computes
  error = current DC time - timestamp (48-bit arithmetic), rounds to an
  integer number of 250 us servo cycles (N).
- Compensation: the N PVT segments starting with the timestamped one get their
  time component adjusted by +/-1 servo cycle each (e.g. error = +3 cycles ->
  segments 5,6,7 run 10.25 ms instead of 10 ms). Negative error -> -1 cycle each.
- A new timestamp received before full compensation discards the remaining
  prior error and uses the new one.
- New read-only 16-bit object 0x2014 = error (in servo cycles) from the most
  recent timestamp; intended for host monitoring/sanity checks.

## Implications to verify in the host code

- Host must periodically emit 0x85 records (DC ns domain, prior-segment
  semantics, lower 48 bits).
- Host should monitor 0x2014 for sync health.
- Segment times are uint-ms on the wire but compensation operates in 250 us
  steps; drift between host clock accounting and drive execution accumulates
  between timestamps -> timestamp cadence matters.
- 48-bit ns wraps every ~3.26 days: wrap handling on both ends.
