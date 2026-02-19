#!/usr/bin/env python3
"""
Generic NetworkTables helper for WPILib simulation testing.

The WPILib simulation starts an NT4 server on localhost:5810. This script connects
as a client and can read, write, or wait on any NT topic. Copilot uses it as a
command-line tool so it never needs to edit this file — all parameters come from
CLI flags and positional arguments.

Install the required library once:
    pip install pyntcore

Usage examples:

  List every published NT topic:
    python3 scripts/nt_reader.py

  Read one or more specific topics:
    python3 scripts/nt_reader.py /AdvantageKit/Intake Linear/Position
    python3 scripts/nt_reader.py /AdvantageKit/IntakeRoller/Velocity /AdvantageKit/Tower/Position

  Write (publish) a value to a topic:
    python3 scripts/nt_reader.py --set true  /SimControl/Enable
    python3 scripts/nt_reader.py --set true  "/SmartDashboard/Intake Linear/Extend/running"
    python3 scripts/nt_reader.py --set 0.0   /Tuning/IntakeRoller/IntakeRPS
    python3 scripts/nt_reader.py --set false /SimControl/Enable

  Poll until a topic reaches an expected value (useful for assertions):
    python3 scripts/nt_reader.py --wait 22.748 --tolerance 0.5 --timeout 15 \\
        "/AdvantageKit/Intake Linear/Position"

Exit codes:
  0 — success (connected; value written, read, or --wait condition met)
  1 — connection timeout or --wait condition not met within --timeout seconds
  2 — pyntcore is not installed
"""

import argparse
import json
import sys
import time

try:
    import ntcore
except ImportError:
    print(
        "ERROR: 'ntcore' module not found.\n"
        "Install it with:  pip install pyntcore",
        file=sys.stderr,
    )
    sys.exit(2)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _connect(host: str, port: int, timeout: float) -> ntcore.NetworkTableInstance:
    """Create an NT4 client instance and block until connected or timeout."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("copilot-nt-tool")
    inst.setServer(host, port)

    print(f"Connecting to NT4 server at {host}:{port} …", file=sys.stderr)
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
    time.sleep(0.25)  # allow topic announcements to arrive
    return inst


def _parse_set_value(raw: str):
    """Parse a --set string into bool, float, or str in that priority order."""
    if raw.lower() == "true":
        return True
    if raw.lower() == "false":
        return False
    try:
        return float(raw)
    except ValueError:
        return raw


def _publish_value(inst: ntcore.NetworkTableInstance, topic_path: str, value) -> None:
    """Publish value to topic_path using the appropriate typed publisher."""
    if isinstance(value, bool):
        pub = inst.getBooleanTopic(topic_path).publish()
        pub.set(value)
    elif isinstance(value, float):
        pub = inst.getDoubleTopic(topic_path).publish()
        pub.set(value)
    else:
        pub = inst.getStringTopic(topic_path).publish()
        pub.set(str(value))
    inst.flush()
    time.sleep(0.5)  # give robot code one or two loops to react
    print(json.dumps({topic_path: value}))


def _get_value(inst: ntcore.NetworkTableInstance, topic_path: str, settle: float = 0.3):
    """Subscribe to a topic, wait settle seconds, and return its current value.

    Tries double first (most robot telemetry), then boolean, then string.
    Uses typed subscribers because getValue().isValid() is unreliable on pyntcore 2026.x.
    """
    _SENTINEL_D = float("-inf")
    sub_d = inst.getDoubleTopic(topic_path).subscribe(_SENTINEL_D)
    sub_b = inst.getBooleanTopic(topic_path).subscribe(None)
    sub_s = inst.getStringTopic(topic_path).subscribe("__NT4_MISSING__")

    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        v = sub_d.get()
        if v != _SENTINEL_D:
            return v
        b = sub_b.get()
        if b is not None:
            return b
        s = sub_s.get()
        if s != "__NT4_MISSING__":
            return s
        time.sleep(0.02)
    return None


def _serialisable(v):
    """Convert non-JSON-serialisable NT values to plain Python."""
    try:
        json.dumps(v)
        return v
    except (TypeError, ValueError):
        return str(v)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generic NetworkTables read/write tool for WPILib simulation testing.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "topics",
        nargs="*",
        metavar="TOPIC",
        help=(
            "NT topic path(s). With --set: exactly one topic to write. "
            "With --wait: exactly one topic to poll. "
            "Without flags: topics to read (omit to list all topics)."
        ),
    )
    parser.add_argument(
        "--set",
        metavar="VALUE",
        dest="set_value",
        help=(
            "Write VALUE to the topic and exit. "
            "'true'/'false' → boolean, numeric string → double, anything else → string."
        ),
    )
    parser.add_argument(
        "--wait",
        type=float,
        metavar="EXPECTED",
        help=(
            "Block until the first TOPIC equals EXPECTED (numeric) within --tolerance. "
            "Requires exactly one topic."
        ),
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.1,
        metavar="TOL",
        help="Acceptable deviation from --wait value (default: 0.1).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        metavar="SECS",
        help="Seconds to wait for connection / --wait condition (default: 10).",
    )
    parser.add_argument(
        "--host",
        default="localhost",
        help="NT4 server hostname (default: localhost).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5810,
        help="NT4 server port (default: 5810).",
    )
    args = parser.parse_args()

    if args.set_value is not None and len(args.topics) != 1:
        parser.error("--set requires exactly one TOPIC.")
    if args.wait is not None and len(args.topics) != 1:
        parser.error("--wait requires exactly one TOPIC.")

    inst = _connect(args.host, args.port, args.timeout)

    try:
        # ── Write mode ────────────────────────────────────────────────────────
        if args.set_value is not None:
            value = _parse_set_value(args.set_value)
            _publish_value(inst, args.topics[0], value)
            return

        # ── Wait/poll mode ────────────────────────────────────────────────────
        if args.wait is not None:
            topic_path = args.topics[0]
            sub_d = inst.getDoubleTopic(topic_path).subscribe(float("-inf"))
            print(
                f"Waiting for {topic_path} ≈ {args.wait} (±{args.tolerance}) …",
                file=sys.stderr,
            )
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                raw = sub_d.get()
                v = raw if raw != float("-inf") else _get_value(inst, topic_path, settle=0.0)
                if v is not None and abs(float(v) - args.wait) <= args.tolerance:
                    print(json.dumps({topic_path: _serialisable(v)}, indent=2))
                    return
                time.sleep(0.05)

            raw = sub_d.get()
            v = raw if raw != float("-inf") else _get_value(inst, topic_path, settle=0.0)
            print(
                f"ERROR: Timed out after {args.timeout}s. Last value: {v}",
                file=sys.stderr,
            )
            print(json.dumps({topic_path: _serialisable(v)}, indent=2))
            sys.exit(1)

        # ── List mode (no topics given) ────────────────────────────────────────
        if not args.topics:
            _discovery_subs = [
                inst.getDoubleTopic(f"/{pfx}/__discovery__").subscribe(float("-inf"))
                for pfx in ["AdvantageKit", "SmartDashboard", "Tuning", "SimControl"]
            ]
            time.sleep(0.5)
            topics = inst.getTopics("")
            names = sorted(t.getName() for t in topics if not t.getName().endswith("__discovery__"))
            print(json.dumps(names, indent=2))
            return

        # ── Read mode (one or more topics) ────────────────────────────────────
        subs = {p: inst.getDoubleTopic(p).subscribe(float("-inf")) for p in args.topics}
        time.sleep(0.4)
        results = {}
        for topic_path in args.topics:
            raw = subs[topic_path].get()
            v = raw if raw != float("-inf") else _get_value(inst, topic_path, settle=0.0)
            results[topic_path] = _serialisable(v)
        print(json.dumps(results, indent=2))

    finally:
        inst.stopClient()


if __name__ == "__main__":
    main()
