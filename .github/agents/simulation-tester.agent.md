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

### Step 2 — Enable the robot

In the Sim GUI **Driver Station** panel:
1. Set mode to **Teleoperated** (or **Autonomous** for auto testing).
2. Click **Enable**.

The robot code will now run its periodic loops and publish values to NetworkTables.

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

## NetworkTables topic reference for this codebase

| NT path prefix | What it contains |
|---|---|
| `/AdvantageKit/{SubsystemName}/` | AdvantageKit-logged sensor inputs (position, velocity, voltage, current, …) |
| `/AdvantageKit/RealOutputs/{SubsystemName}/` | AdvantageKit-logged robot outputs in SIM mode |
| `/Tuning/{SubsystemName}/{param}` | `LoggedTunableNumber` values (PID gains, velocity setpoints, etc.) — only published when `Constants.tuningMode = true` |
| `/SmartDashboard/` | Standard WPILib dashboard values |

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
