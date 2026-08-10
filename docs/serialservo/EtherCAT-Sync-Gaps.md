# EtherCAT X/Y sync and tracking-fidelity analysis — findings and agenda

Analysis of branch `serialservo-tmc4671`, 2026-08-10, against the
production targets: 80 m/s^2, 4 m/s, zero mid-move tracking deviation,
50 nm load-encoder counts, dual-axis Copley over EtherCAT.  Companion:
`copley-508-notes.md` (the 5.08 firmware features).  File:line
references are to this branch.

## Headline

**The Copley 5.08 0x85 timestamp feature is not implemented.**  No
0x85 record is ever built, and `cp_f_command_reset_step_clock`
(klippy/chelper/command.c:475-486) is a stub whose TODO names exactly
this feature ("should use adapted copley firmware and send a segment
representing the absolute time of the following step").  The error
object 0x2014 is neither PDO-mapped nor SDO-readable (no SDO upload
path exists).  Consequently the PVT stream has no absolute-time
anchor: the drive free-runs on its own 250 us crystal from a one-shot
buffer-level start trigger with +-10 ms granularity — up to 40 mm of
schedule offset versus Z/extruder at 4 m/s, and 1 servo cycle (250 us)
is already 1.0 mm of trajectory-time error at 4 m/s.

## Overall verdicts

Time-sync chain: sound up to the host boundary (print_time to
req_clock is exact; compressor clock bookkeeping is full-precision;
one CLOCK_MONOTONIC_RAW domain; the per-cycle DC datagrams do steer
the drive's ESC system time toward host monotonic even with
assign_activate=0 — only the SYNC0-pulse half is inert).  Fully
open-loop beyond it: playback pacing is the drive crystal consuming
ms-quantized durations.  Wire-time truncation made the drive run AHEAD
of host time at roughly 1-3 ms/s (10x crystal drift) — draining the
~260 ms buffer into a hard mid-print stop every few minutes with X/Y
leading Z/E by up to ~200 ms.  The truncation half of that is FIXED on
this branch (round + residual carry); the anchor half needs 0x85.

Decomposition math: the scheme is sound and worth keeping — 10 ms PVT
sampling per se loses nothing (each trapq phase is an exact quadratic;
segment boundaries coincide with sample instants; linear-velocity
interpolation through exact endpoint P/V pairs reproduces the
quadratic identically).  Every premise had a violation; state after
this branch's fixes:

| Premise | Status |
|---|---|
| Planner/trapq consistency | FIXED: per-phase accels in _process_moves + extruder (was +-2 mm inter-move jumps, 40-65 mm/s junction discontinuities) |
| Wire time == host time | FIXED host-side: llround + sub-ms carry, no 0 ms records; absolute anchor still needs 0x85 |
| Sub-count rounding | FIXED: llrint on P/V (was 1-count truncation bias = 50 nm) |
| Hold segments | FIXED: velocity 0, wire time mirrored into time_table (was stale V up to 1.6 m/s during idle) |
| 24-bit wire range | GUARDED: loud error instead of silent wrap; real fix is the record-format decision below |
| Short moves (<2 mm @ 4 m/s) | OPEN (planner design — see agenda) |
| End-of-move pose transmission | OPEN (couples to record semantics — see agenda) |
| Drive honors velocity fields | OPEN (bench question, load-bearing) |

## Agenda (decisions needed, ranked)

1. **0x85 implementation** (needs Nicola + drive at 5.08).  Emit one
   anchor at each motion-burst start plus periodic anchors.  Use
   `ecrt_master_reference_clock_time` (never raw host time) with
   masked 48-bit arithmetic (wrap ~3.26 days).  Rebuild time_table +
   seq_num atomically on any resync.  Each 0x85 record displaces a PVT
   segment in the single mapped 0x2010 slot — co-design with item 3.
   Precondition: verify the drive's PVT engine reads DC system time
   with assign_activate=0, and whether IgH initializes the slave's
   System Time Offset register without DC configured; if not,
   assign_activate must become the Copley-documented value (0x300)
   with DC-aligned cyclic wakeups.
2. **Record format / velocity units** (needs Copley + bench).  Under
   format-0 units (0.1 counts/s at 20,000 counts/mm) 4 m/s wraps the
   24-bit velocity field; format 1 (10 counts/s) covers 4 m/s with
   ~5% headroom.  The deployed velocity_scaling=100 is dimensionally
   wrong under either.  Also confirm whether the drive honors the
   velocity field with 0x60C0=-2 (if it interpolates position-linear
   chords instead, mid-segment error is a*T^2/8 = 1.0 mm at 80 m/s^2
   / 10 ms — the whole fidelity claim rides on this), and whether
   format 4 (32-bit absolute) is the intended post-homing rebase.
3. **Throughput ceiling** (needs Nicola).  One segment per axis per
   10 ms cycle = 100 seg/s cannot feed any path whose average segment
   is under 10 ms; fine G-code at 4 m/s needs 500-1000 seg/s.
   Scaffolding for multiple slots exists (movedata[], dead
   MAX_CYCLE_SEGMENTS).  Options: multiple 0x2010 PDO instances per
   axis, shorter cycle, or both.
4. **Planner quantization strategy** (needs Nicola — reverses part of
   his design).  The 1 ms set_junction rounding is neither necessary
   nor sufficient for integer-ms wire segments (flush fragments break
   ms alignment anyway; the Z path proves the planner needs no
   rounding).  Worse, moves under 2 mm at 4 m/s round to ZERO time and
   vanish from the trapq (arcs become instantaneous multi-mm jumps);
   2-4 mm moves floor to 1 ms with cruise_v rescaled above the machine
   maximum.  Proposed target state: exact planner (drop rounding),
   ms-grid enforcement + residual carry in the ethercat sampler
   (carry half exists now), 0x85 anchoring for the remainder.
5. **End-of-move pose / hold design** (needs Nicola + bench).  The
   sampler emits window-START states and never transmits the profile
   endpoint; with gen_steps_pre/post_active unset, every decel-to-idle
   parks the axis 0.5*a*dt^2 = 4-16 mm short at 80 m/s^2 until the
   next motion (then a catch-up spike).  The Z solver samples window
   ENDS and does not have this defect.  Fix couples to the record
   start-vs-end semantics: the Copley manual supports START sampling
   for format 0, so the likely fix is appending a final
   endpoint-state record per stream end (and making the filler use
   it), plus a zero-time terminator record at intentional stream ends
   (the documented mechanism, never sent) instead of the host-side
   hard stop at fill<=4 that abandons up to 160 mm of buffered path.
6. **Error-recovery protocol** (needs Nicola).  CLEAR_ERRORS written
   into movedata can be clobbered by the same cycle's segment write;
   seq resync leaves time_table under the old numbering (stale by up
   to 320 ms); the filler's partial byte0 write can fabricate
   commands.  The manual's NOP command (code 4) exists to park the
   slot and is unused.  Decide: dedicated command slot or NOP-parking,
   halt segment TX while an error is latched, rebuild time_table on
   resync.  Related: FIRMWARE_RESTART leaves stale seq_num/time_table
   in the static ethercatdata versus an SDO-cleared drive buffer —
   guaranteeing one recovery cycle per restart.
7. **Observability** (cheap, do with any drive session).  Map 0x2014/
   0x2814 into TxPDO 0x1A01 (16 bits each) — the drift instrument for
   everything above.  Add an SDO upload path (all four current SDO
   requests are writes).
8. **Cyclic thread pacing** (bench).  waketime = eventtime + sync0_ct
   accumulates wakeup latency as phase creep (SCHED_FIFO priority 1);
   at 100% slot utilization creep converts to buffer drain.  Fix is
   waketime += sync0_ct with bounded catch-up — verify on the bench
   with the drive, together with item 1.
9. **Homing frame convention** (decide).  Stream correctness silently
   requires drive home offset == position_endstop*scaling; nothing
   enforces or checks it.  Options: format-4 rebase record after
   homing, SDO readback check, or documented invariant +
   COE_CMD_RESET_SEGMENT_ID at homing.
10. Small items: SDO request state never polled (fire-and-forget can
    silently drop the buffer-clear); the serial-motivated clocksync
    conservative bias (+1 ms / -3 sigma) is an uncontrolled component
    of any future 0x85 stamp; msgpool malloc fallback runs in the RT
    thread beyond 512 in-flight segments and out-of-order frees
    degrade the pool permanently (matters once the throughput ceiling
    is raised).

## Verified correct (do not re-litigate)

- Junction velocity continuity in the planner v2 domain is exact.
- Within-move sampling is exact; phase boundaries always coincide with
  sample instants; remainder windows merge into [dt, 2dt).
- Host-side clock accounting does not accumulate error (anchored to
  absolute pose->time each append).
- X and Y share one drive clock: crystal drift is common-mode for the
  XY path shape; it is differential only versus Z/extruder.
- set_junction's cruise_v correction formula is exact within its
  chord model (the prior audit's suspicion of the formula itself was
  wrong — the inconsistency was with trapq's integration, fixed on
  this branch).
- Interpolation sub-mode -2 is configured explicitly at pre-op; the
  0x2011/0x2012 buffer status flow control follows Copley practice.

## Bench-test checklist (first drive session)

1. Read back 0x60C0/0x60C4; stream a deliberately parabolic profile
   and compare actual-position PDO against the commanded quadratic
   (settles the velocity-honoring question, item 2).
2. Measure true count scaling (commanded vs encoder counts).
3. Verify DC system-time convergence with assign_activate=0 (read
   0x0910 twice over a known interval).
4. Verify segment-ID behavior across the 0x60C4:06 buffer clear.
5. Send a single 0x85 anchor, read 0x2014 — measure real accumulated
   drift over a long stream.
