# CAN dropout diagnostics

`frc.robot.diagnostics.CanDropoutDiagnostics` is a logger for the swerve CANcoders dropping off the bus.
It's controlled by `FeatureSwitches.CAN_DROPOUT_DIAGNOSTICS`, which defaults to on.

## What it does

The logger uses its own device objects with the same CAN IDs, so it never touches the drivetrain's signals or
changes any update rate. It only reads frames the devices already broadcast. The one exception is that at boot,
it records every device's sticky faults and then clears them. After that, any sticky fault it logs happened
during this run.

## What it watches

- **Devices:** 4 CANcoders, 8 swerve Talons, the Pigeon and 9 mechanism Talons.
- **Per-device data:**
  - frame arrival gaps
  - dropouts and recoveries (no frame for 200 ms)
  - supply voltage and current
  - live and sticky faults
  - CANcoder magnet health
  - firmware version
- **Faults that point to a cause:**
  - `BootDuringEnable` on a CANcoder, or `RemoteSensorReset` on a steer Talon, means the encoder rebooted
    (a power problem).
  - `RemoteSensorDataInvalid` on a steer Talon means it lost its encoder (this is the chatter).
- **Bus data, from both the roboRIO and Phoenix:**
  - utilization
  - receive and transmit error counters (`REC`/`TEC`), with a log line whenever they cross the warning (96)
    or error-passive (128) level
  - TX-full counts
  - bus-off counts
- **Power and timing:**
  - battery voltage and roboRIO brownouts
  - PDH voltage, total current and faults
  - robot loop timing

## Output

- **Text log (paste this to Claude):** `/home/lvuser/logs/candiag/candiag_<date>_<n>.txt`. One file per
  code start. It contains:
  - a boot snapshot
  - a timestamped event line for every dropout, recovery, fault, error-level change, brownout and note
  - a status line every second while enabled, and every 5 s while disabled
  - per-loop "hr" lines for the first 3 s after each enable
  - a summary table with cause hints after every disable
- **AdvantageKit:** everything also goes under `CanDiag/` in the normal `.wpilog`, for graphing in
  AdvantageScope.
- **Dashboard:** `CanDiag/Down` shows which devices are missing right now, and `CanDiag/File` shows the log path.

## Stamping the log during tests

In SmartDashboard or Elastic, type text into `CanDiag/Note` (for example `wiggling FL encoder plug`), or toggle
`CanDiag/Mark`. The note is written into the log with the current bus state.

## Suggested test session (one code start, robot on blocks)

1. Power on and wait about 10 s while disabled.
2. Enable teleop for 15 s without touching the sticks, then disable.
3. Enable and drive and steer for 15 s, then disable.
4. Enable, and add a note before wiggling each item for about 5 s: each CANcoder's connector, the mini power
   module's terminals, and the module's feed wire. Then disable.
5. Bus-load A/B test: set `SKIP_SWERVE_POWER_TELEMETRY = true`, redeploy, and repeat steps 2–3.

## Pulling the log

Connect to the robot and run the VS Code task `PullCanDiagLogs`, or run this from the repo root:

```
python tools/can-debug/pull_can_logs.py            # candiag_*.txt into can-debug-logs/
python tools/can-debug/pull_can_logs.py --wpilog 1 # plus the newest .wpilog
```

`can-debug-logs/` is committed (not git-ignored), so commit the logs and push them for review. The roboRIO's
`lvuser` has an empty password: if scp prompts for one, press Enter.
