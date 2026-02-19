#!/usr/bin/env python3
"""
Read values from NetworkTables while a WPILib simulation (or real robot) is running.

The WPILib simulation starts an NT4 server on localhost:5810. This script connects
as a client, optionally waits until a topic reaches an expected value, and then
prints the results as JSON so Copilot (or any other caller) can parse them easily.

Install the required library once:
    pip install pyntcore

Usage examples:

  List every published NT topic:
    python3 scripts/nt_reader.py

  Read one or more specific topics:
    python3 scripts/nt_reader.py /AdvantageKit/IntakeRoller/velocity
    python3 scripts/nt_reader.py /Tuning/IntakeRoller/IntakeRPS /AdvantageKit/IntakeRoller/velocity

  Poll until /AdvantageKit/IntakeRoller/velocity equals 10.0 (within ±0.5), timeout 5 s:
    python3 scripts/nt_reader.py --wait 10.0 --tolerance 0.5 --timeout 5 /AdvantageKit/IntakeRoller/velocity

Exit codes:
  0 — success (connected and all requested values read / --wait condition met)
  1 — connection timeout or --wait condition not met within --timeout seconds
  2 — robotpy-ntcore is not installed
"""

import argparse
import json
import sys
import time

# ---------------------------------------------------------------------------
# Friendly error if robotpy-ntcore is missing
# ---------------------------------------------------------------------------
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
    """Create an NT4 client instance and wait until it connects."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("copilot-nt-reader")
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
    # Give the server a moment to push topic announcements.
    time.sleep(0.25)
    return inst


def _subscribe_double(inst: ntcore.NetworkTableInstance, topic_path: str) -> "ntcore.DoubleSubscriber":
    """Create and return an active double subscriber for topic_path."""
    return inst.getDoubleTopic(topic_path).subscribe(float("-inf"))


def _read_subscriber(sub: "ntcore.DoubleSubscriber") -> float | None:
    """Read the latest value from a double subscriber, returning None if not yet received."""
    v = sub.get()
    return v if v != float("-inf") else None


def _get_value(inst: ntcore.NetworkTableInstance, topic_path: str, settle: float = 0.3):
    """Subscribe to a topic, wait ``settle`` seconds, and return its current value.

    Creates a typed double subscriber (the most common type for robot telemetry)
    and falls back to boolean and string if the topic is not a double.

    Uses typed subscribers rather than getValue().isValid() because getValue()
    returns an 'invalid' object on pyntcore 2026.x even when the topic has a value.
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
    """Convert non-JSON-serialisable NT values (e.g. numpy arrays) to plain Python."""
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
        description="Read NetworkTables values from a running WPILib simulation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "topics",
        nargs="*",
        metavar="TOPIC",
        help=(
            "NT topic path(s) to read, e.g. /AdvantageKit/IntakeRoller/velocity. "
            "If omitted, all published topics are listed."
        ),
    )
    parser.add_argument(
        "--wait",
        type=float,
        metavar="EXPECTED",
        help=(
            "Block until the first TOPIC equals EXPECTED (numeric) within --tolerance. "
            "Requires exactly one topic to be specified."
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
        default=5.0,
        metavar="SECS",
        help="Seconds to wait for connection and --wait condition (default: 5).",
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

    if args.wait is not None and len(args.topics) != 1:
        parser.error("--wait requires exactly one TOPIC to be specified.")

    inst = _connect(args.host, args.port, args.timeout)

    try:
        if not args.topics:
            # List all topics.
            # NT4 clients only receive topic announcements for subscribed prefixes.
            # Subscribe to the common WPILib root tables so the server sends their topics.
            _discovery_subs = [
                inst.getDoubleTopic(f"/{pfx}/__discovery__").subscribe(float("-inf"))
                for pfx in ["AdvantageKit", "SmartDashboard", "Tuning", "SimControl"]
            ]
            time.sleep(0.4)
            topics = inst.getTopics("")
            names = sorted(t.getName() for t in topics if not t.getName().endswith("__discovery__"))
            print(json.dumps(names, indent=2))
            return

        if args.wait is not None:
            # Create a subscription upfront so the server starts sending values immediately.
            topic_path = args.topics[0]
            sub_d = inst.getDoubleTopic(topic_path).subscribe(float("-inf"))
            print(
                f"Waiting for {topic_path} ≈ {args.wait} (±{args.tolerance}) …",
                file=sys.stderr,
            )
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                raw = sub_d.get()
                v = raw if raw != float("-inf") else None
                if v is None:
                    v = _get_value(inst, topic_path, settle=0.0)
                if v is not None and abs(float(v) - args.wait) <= args.tolerance:
                    print(json.dumps({topic_path: _serialisable(v)}, indent=2))
                    return
                time.sleep(0.05)

            # Timed out — print whatever we have so the caller can see the actual value.
            raw = sub_d.get()
            v = raw if raw != float("-inf") else _get_value(inst, topic_path, settle=0.0)
            print(
                f"ERROR: Timed out after {args.timeout}s. "
                f"Last value: {v}",
                file=sys.stderr,
            )
            print(json.dumps({topic_path: _serialisable(v)}, indent=2))
            sys.exit(1)

        else:
            # Read each requested topic once.
            # Create all subscriptions first so the server starts sending values,
            # then wait once for all of them to arrive.
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
