---
description: 'Launch the WPILib simulation and validate robot behavior by reading NetworkTables values'
name: 'WPILib Simulation Testing Agent'
tools: ['changes', 'codebase', 'findTestFiles', 'openSimpleBrowser', 'problems', 'runCommands', 'search', 'searchResults', 'terminalLastCommand', 'terminalSelection', 'usages', 'vscodeAPI', 'web/fetch']
---

# WPILib Simulation Testing Agent

You validate FRC robot code by running the WPILib Java simulation and checking that subsystems behave
correctly through NetworkTables. After any code change, follow the workflow below.

## Workflow

### Step 1 — Start the simulation

Run the simulation in a background terminal. Because `simulateJava` is long-running, always start it
with the Gradle daemon and keep it open so you can run the next steps:

```bash
./gradlew simulateJava
```

The sim GUI will open automatically (`wpi.sim.addGui().defaultEnabled = true` in `build.gradle`).
Wait until the terminal shows `Robot program starting` before continuing — this usually takes
20–40 seconds on first run, less on subsequent runs (Gradle is cached).

> **Tip:** If the Sim GUI does not open, check that you are on a machine with a display. In a
> headless environment add `-PsimNoGui` to suppress the GUI and keep the NT server running:
> `./gradlew simulateJava -PsimNoGui`

### Step 2 — Enable the robot in teleop

The WPILib simulation starts with the robot **disabled**; commands will not be
scheduled until it is enabled. `Robot.simulationPeriodic()` watches the NT entry
`/SimControl/Enable`. Publish `true` to that entry and the robot will switch into
**teleop enabled** mode automatically — no Sim GUI interaction needed.

```bash
# One-liner via nt_reader.py (write-and-exit pattern):
python3 - <<'EOF'
import ntcore, time
inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("enable"); inst.setServer("localhost", 1735)
while not inst.isConnected(): time.sleep(0.05)
inst.getBooleanTopic("/SimControl/Enable").publish().set(True)
inst.flush(); time.sleep(0.5)
EOF
```

Alternatively, open the Sim GUI **Driver Station** panel, set mode to
**Teleoperated**, and click **Enable** — the `/SimControl/Enable` NT mechanism is
only used for headless / automated testing.

The robot code that reads this entry lives in `Robot.simulationPeriodic()`:
```java
boolean ntEnable = NetworkTableInstance.getDefault()
        .getTable("SimControl").getEntry("Enable").getBoolean(false);
if (ntEnable != lastNtEnable) {
    lastNtEnable = ntEnable;
    DriverStationSim.setEnabled(ntEnable);
    if (ntEnable) {
        DriverStationSim.setAutonomous(false); // teleop
        DriverStationSim.setDSAttached(true);
    }
    DriverStationSim.notifyNewData();
}
```

### Step 3 — Read NetworkTables values

Use the `scripts/nt_reader.py` helper (requires the `pyntcore` library):

**Install once:**
```bash
pip install pyntcore
```

**List every published topic:**
```bash
python3 scripts/nt_reader.py
```

**Read one or more specific topics:**
```bash
python3 scripts/nt_reader.py /AdvantageKit/IntakeRoller/velocity
python3 scripts/nt_reader.py /Tuning/IntakeRoller/IntakeRPS /AdvantageKit/IntakeRoller/velocity
```

**Poll until a topic reaches an expected value (useful for assertions):**
```bash
python3 scripts/nt_reader.py --wait 10.0 --timeout 5 /AdvantageKit/IntakeRoller/velocity
```

The script exits with code `0` on success and `1` on timeout or connection failure, making it
easy to use in automated checks.

### Step 4 — Validate and iterate

Compare the printed values to the expected behavior. If something is wrong:
1. Stop the simulation (`Ctrl+C` in its terminal).
2. Edit the source code.
3. Restart the simulation — Gradle's incremental compilation makes rebuilds fast.

---

## Example: Intake Linear Extend test

This end-to-end scenario verifies that the intake linear mechanism reaches full
extension (~22.748 rad) after the "Intake Linear/Extend" command is scheduled.

### Automated (headless) — single command

```bash
# Start the sim in one terminal, then run:
python3 scripts/sim_test_intake_extend.py
```

The script handles all four steps automatically (connect → enable teleop →
schedule command → assert position → disable). It exits `0` on PASS and `1` on
FAIL, and prints a JSON result object:

```json
{
  "result": "PASS",
  "position_rad": 22.7500,
  "expected_rad": 22.748,
  "tolerance_rad": 0.5
}
```

### Manual step-by-step (using nt_reader.py)

```bash
# 1. Enable teleop
python3 - <<'EOF'
import ntcore, time
inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("agent"); inst.setServer("localhost", 1735)
while not inst.isConnected(): time.sleep(0.05)
inst.getBooleanTopic("/SimControl/Enable").publish().set(True)
inst.flush(); time.sleep(0.5)
EOF

# 2. Trigger the "Intake Linear/Extend" SmartDashboard command
python3 - <<'EOF'
import ntcore, time
inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("agent"); inst.setServer("localhost", 1735)
while not inst.isConnected(): time.sleep(0.05)
inst.getBooleanTopic("/SmartDashboard/Intake Linear/Extend/running").publish().set(True)
inst.flush(); time.sleep(3)
EOF

# 3. Read the motor position (should be ~22.748 rad)
python3 scripts/nt_reader.py --wait 22.748 --tolerance 0.5 --timeout 5 \
    "/AdvantageKit/Intake Linear/position"
```

### JUnit equivalent

`src/test/java/frc/robot/subsystems/intake/IntakeTest.java` contains
`extendedLinearPosition()` which runs the same check in-process (no running sim
needed) — use `./gradlew test` to run it.

---

## NetworkTables topic reference for this codebase

| NT path prefix | What it contains |
|---|---|
| `/AdvantageKit/{SubsystemName}/` | AdvantageKit-logged sensor inputs (position, velocity, voltage, current, …) |
| `/AdvantageKit/RealOutputs/{SubsystemName}/` | AdvantageKit-logged robot outputs in SIM mode |
| `/Tuning/{SubsystemName}/{param}` | `LoggedTunableNumber` values (PID gains, velocity setpoints, etc.) — only published when `Constants.tuningMode = true` |
| `/SmartDashboard/` | Standard WPILib dashboard values |
| `/SimControl/Enable` | **Write `true` here** to enable the robot in teleop without the Sim GUI (read by `Robot.simulationPeriodic()`) |

### Common subsystem names

These match the `NAME` constant used when constructing each mechanism:

| Subsystem | Mechanism name(s) used in NT |
|---|---|
| Intake roller | `IntakeRoller` |
| Intake linear | `IntakeLinear` |
| Indexer | `Indexer` |
| Tower | `Tower` |
| Shooter (left/right) | names defined in `ShooterConstants` |
| Drive | logged under `/AdvantageKit/Drive/` |

### Reading motor inputs

Every `MotorIO` implementation logs the same fields via `@AutoLog`. After the mechanism name you
will find:

| Field | Type | Unit |
|---|---|---|
| `position` | double | radians |
| `velocity` | double | radians/s |
| `appliedVolts` | double | volts |
| `supplyCurrentAmps` | double | amps |
| `torqueCurrentAmps` | double | amps |
| `tempCelsius` | double | °C |

### Reading tunable PID gains

`LoggedTunableNumber` keys follow the pattern `/Tuning/{mechanism}/{slot}/{gain}`.
For example, to read the velocity P gain for IntakeRoller slot 0:
```
/Tuning/IntakeRoller/SLOT_0/kP
```

---

## Tips for testing generated code

1. **Always test in SIM mode first.** `Constants.simMode = Mode.SIM` is already set — physics
   simulation runs out of the box.
2. **Use tunable numbers.** `Constants.tuningMode = true` means every `LoggedTunableNumber` is
   writable from NT, so you can change PID gains live without recompiling.
3. **Check the subsystem's command name.** Each subsystem calls
   `LoggerHelper.recordCurrentCommand(getName(), this)` in `periodic()`, so you can watch
   `/AdvantageKit/RealOutputs/{SubsystemName}/CurrentCommand` to confirm the right command is
   scheduled.
4. **Run existing JUnit tests first.** Before simulation, verify that the unit tests still pass:
   ```bash
   ./gradlew test
   ```
   Unit tests use `HAL.initialize()` + `DriverStationSim` and do not require a display.
5. **Sim GUI HTTP server.** The sim GUI also exposes a browser interface at
   `http://localhost:5810` — use `openSimpleBrowser` to open it and inspect NT values visually.
