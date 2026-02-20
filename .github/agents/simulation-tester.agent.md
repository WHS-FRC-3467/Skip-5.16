---
description: 'Launch the WPILib simulation and validate robot behavior by reading NetworkTables values'
name: 'WPILib Simulation Testing Agent'
tools: ['changes', 'codebase', 'findTestFiles', 'openSimpleBrowser', 'problems', 'runCommands', 'search', 'searchResults', 'terminalLastCommand', 'terminalSelection', 'usages', 'vscodeAPI', 'web/fetch']
---

# WPILib Simulation Testing Agent

You validate FRC robot code by running the WPILib Java simulation and checking that subsystems
behave correctly through NetworkTables. **Choose the right path for your environment:**

| Environment | How to run simulation | How to interact with NT |
|---|---|---|
| **Developer machine** (VS Code, display available) | `./gradlew simulateJava` in a terminal | `python3 scripts/nt_reader.py` directly |
| **Copilot sandbox / CI** (no display, FRC Maven DNS-blocked) | Trigger `.github/workflows/simulate.yml` in GitHub Actions | Pass test parameters as `workflow_dispatch` inputs to the workflow |

---

## Path A — Developer machine (local)

### Step 1 — Start the simulation

```bash
./gradlew simulateJava
```

Wait until `Robot program starting` appears (20–60 s first run; faster after Gradle caches).

### Step 2 — Install the NT helper (once)

```bash
pip install pyntcore
```

### Step 3 — Use `scripts/nt_reader.py` for all NT operations

`nt_reader.py` is your single generic tool — no editing required, all parameters come from
CLI flags. It prints JSON to stdout and exits with code `0` (success) or `1` (fail).

**List every published topic** (useful for finding exact spellings):
```bash
python3 scripts/nt_reader.py
```

**Read one or more topics:**
```bash
python3 scripts/nt_reader.py "/AdvantageKit/<MechanismName>/<Field>"
python3 scripts/nt_reader.py "/AdvantageKit/<MechanismName>/Position" "/AdvantageKit/<OtherMechanism>/Velocity"
```

**Write a value to a topic** (`--set`):
```bash
# Enable the robot in teleop
python3 scripts/nt_reader.py --set true /SimControl/Enable

# Schedule a SmartDashboard command
python3 scripts/nt_reader.py --set true "/SmartDashboard/<SubsystemName>/<CommandName>/running"

# Change a tunable number
python3 scripts/nt_reader.py --set 50.0 "/Tuning/<MechanismName>/<param>"

# Disable the robot
python3 scripts/nt_reader.py --set false /SimControl/Enable
```

**Poll until a topic reaches an expected value** (`--wait`):
```bash
python3 scripts/nt_reader.py \
    --wait <EXPECTED> --tolerance <TOL> --timeout <SECS> \
    "/AdvantageKit/<MechanismName>/<Field>"
```

### Step 4 — Generic test pattern (any subsystem, any command)

```bash
# 1. Enable teleop
python3 scripts/nt_reader.py --set true /SimControl/Enable

# 2. Schedule the command (SmartDashboard key registered in RobotContainer)
python3 scripts/nt_reader.py --set true "/SmartDashboard/<SubsystemName>/<CommandName>/running"

# 3. Assert the expected NT value
python3 scripts/nt_reader.py --wait <EXPECTED> --tolerance <TOL> --timeout <SECS> \
    "/AdvantageKit/<MechanismName>/<Field>"
```

---

## Path B — Copilot sandbox / CI (no local simulation)

When `./gradlew simulateJava` cannot run locally (FRC Maven DNS-blocked, no display), the
simulation runs in GitHub Actions via `.github/workflows/simulate.yml`.

The workflow is fully parameterized — pass any command topic, assert topic, and expected value
as `workflow_dispatch` inputs via the `gh` CLI. No files need to be edited.

```bash
gh workflow run simulate.yml \
  --ref <branch> \
  --field command_topic="/SmartDashboard/<SubsystemName>/<CommandName>/running" \
  --field assert_topic="/AdvantageKit/<MechanismName>/<Field>" \
  --field assert_value="<expected>" \
  --field assert_tolerance="<tolerance>" \
  --field assert_timeout="<seconds>"
```

After triggering, monitor the run:
```bash
gh run list --workflow=simulate.yml --limit 5
gh run watch   # stream logs of the most recent run
```

The "Assert NT topic reaches expected value" step exits `0` on success and `1` on timeout.

---

## NT topic reference

| NT path | What it contains |
|---|---|
| `/SimControl/Enable` | **Write `true`** to enable robot in teleop; `false` to disable |
| `/SmartDashboard/<Name>/running` | **Write `true`** to schedule a command registered with `SmartDashboard.putData()` |
| `/AdvantageKit/<MechanismName>/Position` | Motor shaft position (radians) |
| `/AdvantageKit/<MechanismName>/Velocity` | Motor shaft velocity (rad/s) |
| `/AdvantageKit/<MechanismName>/AppliedVolts` | Applied voltage (V) |
| `/AdvantageKit/<MechanismName>/SupplyCurrentAmps` | Supply current (A) |
| `/AdvantageKit/RealOutputs/<Name>/CurrentCommand` | Active command name (string) |
| `/Tuning/<MechanismName>/<param>` | `LoggedTunableNumber` values — writable when `tuningMode = true` |

### Common mechanism names

| Subsystem | Mechanism name in NT |
|---|---|
| Intake linear | `Intake Linear` |
| Intake roller | `IntakeRoller` |
| Indexer | `Indexer` |
| Tower | `Tower` |
| Drive | `Drive` |

---

## Tips

1. **List topics first** when working with an unfamiliar subsystem:
   `python3 scripts/nt_reader.py` shows every published path.
2. **AdvantageKit capitalises field names** (`Position`, `Velocity`, `AppliedVolts`, …).
3. **Commands need teleop enabled** — the robot ignores `running` writes while disabled.
4. **Increase `--timeout`** for slow-moving mechanisms (e.g., `--timeout 30` for an elevator).
5. **Check the active command** with:
   ```bash
   python3 scripts/nt_reader.py "/AdvantageKit/RealOutputs/<SubsystemName>/CurrentCommand"
   ```
