#!/usr/bin/env python3
"""
End-to-end simulation test: extend the intake linear mechanism and verify the
motor position reaches ~22.748 radians.

This script exercises the same workflow described in the simulation-tester agent:

  1. Connect to the NT4 server at localhost:5810 (started by the simulation).
  2. Enable the robot in teleop by writing true to /SimControl/Enable.
     Robot.simulationPeriodic() reads this entry and calls DriverStationSim.setEnabled(true).
  3. Trigger the "Intake Linear/Extend" SmartDashboard command by writing true to
     /SmartDashboard/Intake Linear/Extend/running.
  4. Poll /AdvantageKit/Intake Linear/position until it reaches ~22.748 rad (MAX_DISTANCE /
     DRUM_RADIUS = 11.375 in / 0.5 in).
  5. Print a JSON result and exit 0 (PASS) or 1 (FAIL).

Usage:
  # Terminal 1 — start the simulation and wait for "Robot program starting":
  ./gradlew simulateJava

  # Terminal 2 — run this test:
  pip install pyntcore
  python3 scripts/sim_test_intake_extend.py

Exit codes:
  0 — PASS: position reached the expected value within the timeout
  1 — FAIL: timeout expired or connection failed
  2 — pyntcore not installed
"""

import argparse
import json
import sys
import time

try:
    import ntcore
except ImportError:
    print(
        "ERROR: 'ntcore' module not found.\nInstall it with:  pip install pyntcore",
        file=sys.stderr,
    )
    sys.exit(2)

# ── Constants ─────────────────────────────────────────────────────────────────

# Motor shaft position (radians) when the intake linear is fully extended.
# Derived from IntakeLinearConstants: MAX_DISTANCE / DRUM_RADIUS = 11.375 in / 0.5 in = 22.75 rad.
# The problem statement uses the rounded value 22.748 rad; both are within tolerance.
EXPECTED_POSITION_RAD = 22.748
POSITION_TOLERANCE_RAD = 0.5  # ±0.5 rad (~0.25 in linear)

# NetworkTables paths ──────────────────────────────────────────────────────────
# Written by this script → read by Robot.simulationPeriodic() to enable the robot in teleop.
NT_SIM_ENABLE = "/SimControl/Enable"

# Written by this script → WPILib Command.initSendable() setter schedules extendIntake().
# Registered in RobotContainer.initializeDashboard():
#   SmartDashboard.putData("Intake Linear/Extend", intake.extendIntake())
NT_COMMAND_RUNNING = "/SmartDashboard/Intake Linear/Extend/running"

# Read by this script ← AdvantageKit NT4Publisher logs MotorInputs.position (radians).
# Published in Mechanism.periodic() via Logger.processInputs("Intake Linear", inputs).
NT_POSITION = "/AdvantageKit/Intake Linear/position"


# ── Helpers ───────────────────────────────────────────────────────────────────


def connect(host: str, port: int, timeout: float) -> ntcore.NetworkTableInstance:
    """Start an NT4 client and block until connected or timeout expires."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("sim-test-intake-extend")
    inst.setServer(host, port)

    print(f"Connecting to NT4 at {host}:{port} …", file=sys.stderr)
    deadline = time.monotonic() + timeout
    while not inst.isConnected():
        if time.monotonic() > deadline:
            print(
                f"ERROR: Could not connect within {timeout}s.\n"
                "Is the simulation running?  Start it with:  ./gradlew simulateJava",
                file=sys.stderr,
            )
            sys.exit(1)
        time.sleep(0.05)

    print("Connected.", file=sys.stderr)
    time.sleep(0.3)  # allow topic announcements to arrive
    return inst


def read_double(sub: "ntcore.DoubleSubscriber") -> float | None:
    """Return the current value from a DoubleSubscriber, or None if not yet received.

    Uses a typed subscriber (created once by the caller) rather than getEntry().getDouble()
    because getEntry() is not guaranteed to have active subscriptions on pyntcore 2026.x.
    """
    v = sub.get()
    return v if v != float("-inf") else None


# ── Main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default="localhost", help="NT4 server hostname")
    parser.add_argument("--port", type=int, default=5810, help="NT4 server port (default: 5810)")
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=15.0,
        metavar="SECS",
        help="Seconds to wait for NT4 connection (default: 15)",
    )
    parser.add_argument(
        "--test-timeout",
        type=float,
        default=8.0,
        metavar="SECS",
        help="Seconds to wait for position to reach target (default: 8)",
    )
    args = parser.parse_args()

    inst = connect(args.host, args.port, args.connect_timeout)

    # Publishers kept alive for the duration of the test.
    enable_pub = inst.getBooleanTopic(NT_SIM_ENABLE).publish()
    cmd_pub = inst.getBooleanTopic(NT_COMMAND_RUNNING).publish()
    # Create the position subscriber immediately so the server starts sending values
    # before we need them.  Using float("-inf") as sentinel (no valid motor angle is -∞).
    pos_sub = inst.getDoubleTopic(NT_POSITION).subscribe(float("-inf"))

    try:
        # ── Step 1: Enable the robot in teleop ────────────────────────────────
        # Robot.simulationPeriodic() watches /SimControl/Enable.  When it changes
        # from false → true it calls:
        #   DriverStationSim.setEnabled(true)
        #   DriverStationSim.setAutonomous(false)   // teleop
        #   DriverStationSim.setDSAttached(true)
        #   DriverStationSim.notifyNewData()
        print("Step 1 — enabling robot in teleop …", file=sys.stderr)
        enable_pub.set(True)
        inst.flush()
        time.sleep(0.5)  # wait one or two robot loops for the enable to take effect

        # ── Step 2: Schedule the "Intake Linear/Extend" command ───────────────
        # SmartDashboard.putData("Intake Linear/Extend", intake.extendIntake()) in
        # RobotContainer registers the command as a Sendable.  WPILib's Command
        # initSendable() binds the NT boolean "running" to command.schedule().
        print("Step 2 — scheduling 'Intake Linear/Extend' command …", file=sys.stderr)
        cmd_pub.set(True)
        inst.flush()

        # ── Step 3: Wait for the motor position to reach ~22.748 rad ─────────
        print(
            f"Step 3 — waiting up to {args.test_timeout}s for "
            f"/AdvantageKit/Intake Linear/position ≈ {EXPECTED_POSITION_RAD} rad …",
            file=sys.stderr,
        )
        deadline = time.monotonic() + args.test_timeout
        last_pos = None
        while time.monotonic() < deadline:
            pos = read_double(pos_sub)
            if pos is not None:
                last_pos = pos
                if abs(pos - EXPECTED_POSITION_RAD) <= POSITION_TOLERANCE_RAD:
                    result = {
                        "result": "PASS",
                        "position_rad": round(pos, 4),
                        "expected_rad": EXPECTED_POSITION_RAD,
                        "tolerance_rad": POSITION_TOLERANCE_RAD,
                    }
                    print(
                        f"\nPASS  position = {pos:.4f} rad  "
                        f"(expected {EXPECTED_POSITION_RAD} ± {POSITION_TOLERANCE_RAD})",
                        file=sys.stderr,
                    )
                    print(json.dumps(result, indent=2))
                    return  # exit via finally → disable robot
            time.sleep(0.05)

        # Timed out
        result = {
            "result": "FAIL",
            "reason": "timeout",
            "last_position_rad": round(last_pos, 4) if last_pos is not None else None,
            "expected_rad": EXPECTED_POSITION_RAD,
            "tolerance_rad": POSITION_TOLERANCE_RAD,
        }
        print(
            f"\nFAIL  timed out after {args.test_timeout}s.  "
            f"Last position: {last_pos}",
            file=sys.stderr,
        )
        print(json.dumps(result, indent=2))
        sys.exit(1)

    finally:
        # ── Step 4: Disable the robot and clean up ────────────────────────────
        print("Step 4 — disabling robot …", file=sys.stderr)
        enable_pub.set(False)
        inst.flush()
        time.sleep(0.1)
        inst.stopClient()


if __name__ == "__main__":
    main()
