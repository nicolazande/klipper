# Shared-PWM fans with independent tachometers (master/slave)

## Why this exists

The controller boards have far fewer PWM-capable pins than fans, so
several fans share one PWM output.  Sharing the PWM is electrically
fine - the danger is losing per-fan feedback: a single stalled fan in
a bank can let a hotend, driver, or LED board build damaging heat
while every other fan on the shared pin spins normally.  Tachometer
inputs are cheap GPIOs, so the design gives every physical fan its own
tach pin while any number of fans share one PWM drive.

The `master_fan` option on `[fan_generic]` implements exactly that:
one section owns the PWM pin (the master), any number of further
sections reference it (slaves) and contribute only their tachometer.
Every fan - master and slaves - shows up individually in the API and
frontends with its own RPM, so a stalled fan is visible (and can be
acted on) even though it has no dedicated PWM.

## How it works

- The master builds the PWM pin, the kick-start logic, the shutdown
  value and the gcode request queue.  Slaves hold a reference to the
  master's PWM object and build only a tachometer.
- Any speed request addressed to a slave (`SET_FAN_SPEED FAN=<slave>`)
  is forwarded to the master, so commanding any fan of a bank changes
  the whole bank - there is one physical speed.
- `get_status` on a slave reports the master's commanded speed
  together with the slave's OWN measured rpm.  Monitoring therefore
  works per physical fan.
- On shutdown the master's `shutdown_speed` applies to the bank (one
  pin, one value).

## Configuration

```
# The master owns the PWM pin (and optional enable pin)
[fan_generic AC_Motor_Cooling_Fan_Right]
pin: PE5
cycle_time: 0.010
tachometer_pin: ^PE7
tachometer_ppr: 2

# Slaves name their master and add only a tachometer
[fan_generic AC_Motor_Cooling_Fan_Left]
master_fan: AC_Motor_Cooling_Fan_Right
tachometer_pin: ^PE8
tachometer_ppr: 2
```

Rules and limitations:

- The master section must appear BEFORE its slaves in printer.cfg
  (the lookup happens while the config is parsed; a wrong order fails
  loudly at startup with "Master fan ... not found").
- Both master and slaves must be `[fan_generic]` sections.  `[fan]`,
  `[heater_fan]` and `[controller_fan]` do not participate.
- PWM-related options in a slave section (`pin`, `max_power`,
  `cycle_time`, `hardware_pwm`, `kick_start_time`, `off_below`,
  `shutdown_speed`) are ignored - the master's values rule the bank.
  Set them only on the master.
- `enable_pin` is rejected on slaves (a slave's speed path never runs,
  so the gate would never open); put it on the master.
- Chained masters (a slave acting as another fan's master) are not
  supported: `master_fan` must name a section without `master_fan`.

## Monitoring stalled fans

The tachometers make failure VISIBLE (`rpm` per fan in
`printer.objects` / Mainsail); nothing in the host acts on a low rpm
by itself yet.  Until a dedicated fan-watchdog exists, pair the bank
with a macro or host-side monitor that checks each fan's rpm against
a minimum whenever the commanded speed is nonzero, e.g. a
`delayed_gcode` loop raising an alert/pausing the print.  This is the
natural next step if fan failure should halt heating autonomously.
