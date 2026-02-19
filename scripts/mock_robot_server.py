#!/usr/bin/env python3
"""
Lightweight mock NT4 server that simulates the robot simulation's behavior for
testing scripts/sim_test_intake_extend.py (and nt_reader.py) without a running
WPILib simulation.

What it does:
  - Starts an NT4 server on port 5810 (the standard WPILib NT4 port)
  - Publishes /AdvantageKit/Intake Linear/position starting at 0.0
  - Watches /SimControl/Enable — when set true, marks the robot as enabled
  - Watches /SmartDashboard/Intake Linear/Extend/running — when set true while
    enabled, ramps the position from 0 → 22.75 rad at 15 rad/s (reaches target
    in ~1.5 s, matching the real ElevatorSim behavior)

Usage (two terminals):
  # Terminal 1 — start the mock server:
  pip install pyntcore
  python3 scripts/mock_robot_server.py

  # Terminal 2 — run the integration test:
  python3 scripts/sim_test_intake_extend.py

The server runs for 60 seconds and then exits automatically.
Press Ctrl+C to stop early.
"""

import threading
import time

try:
    import ntcore
except ImportError:
    import sys

    print(
        "ERROR: 'ntcore' module not found.\nInstall it with:  pip install pyntcore",
        file=sys.stderr,
    )
    sys.exit(2)

# ── Physics constants (mirrors IntakeLinearConstants) ─────────────────────────

FINAL_POSITION_RAD = 22.75  # MAX_DISTANCE / DRUM_RADIUS = 11.375 in / 0.5 in
RAMP_RATE_RAD_PER_S = 15.0  # reach target in ~1.5 s
UPDATE_HZ = 50  # 50 Hz physics update rate

# ── NetworkTables paths ───────────────────────────────────────────────────────

NT_POSITION = "/AdvantageKit/Intake Linear/position"
NT_SIM_ENABLE = "/SimControl/Enable"
NT_CMD_RUNNING = "/SmartDashboard/Intake Linear/Extend/running"


def run_server(duration_s: float = 60.0) -> None:
    inst = ntcore.NetworkTableInstance.create()
    inst.startServer()

    # Publishers (robot → clients)
    pos_pub = inst.getDoubleTopic(NT_POSITION).publish()
    pos_pub.set(0.0)
    inst.flush()

    # Subscribers (clients → robot)
    enable_sub = inst.getBooleanTopic(NT_SIM_ENABLE).subscribe(False)
    cmd_sub = inst.getBooleanTopic(NT_CMD_RUNNING).subscribe(False)

    position = 0.0
    stop_event = threading.Event()

    def physics_loop() -> None:
        nonlocal position
        dt = 1.0 / UPDATE_HZ
        while not stop_event.wait(dt):
            enabled = enable_sub.get()
            cmd = cmd_sub.get()
            if enabled and cmd and position < FINAL_POSITION_RAD:
                position = min(position + RAMP_RATE_RAD_PER_S * dt, FINAL_POSITION_RAD)
                pos_pub.set(position)
                inst.flush()

    t = threading.Thread(target=physics_loop, daemon=True)
    t.start()

    print(
        f"Mock robot NT4 server started on port 5810.\n"
        f"  Publish {NT_POSITION} (starting at 0.0 rad)\n"
        f"  Watch {NT_SIM_ENABLE} to enable robot\n"
        f"  Watch {NT_CMD_RUNNING} to start extending\n"
        f"  Target position: {FINAL_POSITION_RAD} rad\n"
        f"Running for up to {duration_s}s — press Ctrl+C to stop early.\n"
    )

    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            time.sleep(1.0)
            enabled = enable_sub.get()
            cmd = cmd_sub.get()
            clients = len(inst.getConnections())
            print(
                f"  clients={clients}  enabled={enabled}  cmd={cmd}  "
                f"position={position:.3f} rad",
                flush=True,
            )
    except KeyboardInterrupt:
        print("\nStopping mock server …")
    finally:
        stop_event.set()
        inst.stopServer()


if __name__ == "__main__":
    run_server()
