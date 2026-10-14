#!/usr/bin/env python3
"""Extra ROS1 observer and exact-header join; gaps are transport proxies, not bridge CPU time."""
import argparse
import csv
import json
import math
import struct
import threading
from pathlib import Path


def serialized_header(data):
    """Read the common ROS1 Header prefix without deserializing image pixels."""
    if len(data) < 16:
        raise ValueError("truncated ROS1 Header")
    sequence, seconds, nanoseconds, size = struct.unpack_from("<IIII", data)
    if nanoseconds >= 1000000000 or size > len(data) - 16:
        raise ValueError("invalid ROS1 Header timestamp or frame length")
    return sequence, seconds * 1000000000 + nanoseconds, bytes(data[16:16 + size]).decode("utf-8")


def clock_stamp(message):
    """Use typed Clock: rospy's internal sim-time subscriber shares this topic transport."""
    stamp = message.clock.to_nsec()
    if not isinstance(stamp, int) or stamp < 0:
        raise ValueError("invalid /clock nanoseconds")
    return stamp


def distribution(values):
    finite = sorted(value for value in values if math.isfinite(value) and value >= 0)
    def percentile(q):
        if not finite:
            return None
        position = (len(finite) - 1) * q
        lo = int(position)
        return finite[lo] + (finite[min(lo + 1, len(finite) - 1)] - finite[lo]) * (position - lo)
    return {"count": len(finite), "negative_count": sum(value < 0 for value in values),
            "nonfinite_count": sum(not math.isfinite(value) for value in values),
            "p50_s": percentile(.5), "p95_s": percentile(.95), "max_s": finite[-1] if finite else None}


def join_rows(probe, timing):
    results = {}
    for stream in ("left", "right"):
        observer = {}
        duplicate_probe = 0
        invalid_clock = 0
        for row in probe:
            if row["stream"] != stream:
                continue
            if row["clock_valid"] != "1":
                invalid_clock += 1
                continue
            stamp = int(row["source_stamp_ns"])
            if stamp in observer:
                duplicate_probe += 1
            else:
                observer[stamp] = int(row["observer_sim_ns"])
        output = {}
        duplicate_output = 0
        for row in timing:
            stamp_field, time_field = stream + "_image_stamp_ns", stream + "_callback_sim_s"
            if not row.get(stamp_field) or not row.get(time_field):
                continue
            stamp, arrival = int(row[stamp_field]), float(row[time_field])
            if not math.isfinite(arrival) or arrival <= 0:
                continue
            if stamp in output:
                duplicate_output += 1
            else:
                output[stamp] = arrival
        lower = max(min(observer), min(output)) if observer and output else None
        upper = min(max(observer), max(output)) if observer and output else None
        common = lower is not None and lower <= upper
        obs_window = {stamp for stamp in observer if common and lower <= stamp <= upper}
        out_window = {stamp for stamp in output if common and lower <= stamp <= upper}
        matched = sorted(obs_window & out_window)
        results[stream] = {"common_source_window_start_ns": lower if common else None,
                          "common_source_window_end_ns": upper if common else None,
                          "paired_count": len(matched), "probe_unique_count": len(observer),
                          "ros2_unique_count": len(output), "probe_duplicate_count": duplicate_probe,
                          "ros2_duplicate_count": duplicate_output, "probe_invalid_clock_count": invalid_clock,
                          "probe_only_common_window_count": len(obs_window - out_window),
                          "ros2_only_common_window_count": len(out_window - obs_window),
                          "source_to_ros1_observer_sim": distribution([(observer[s] - s) / 1e9 for s in matched]),
                          "ros1_observer_to_ros2_callback_sim_proxy": distribution([output[s] - observer[s] / 1e9 for s in matched]),
                          "source_to_ros2_callback_sim": distribution([output[s] - s / 1e9 for s in matched])}
    return {"streams": results, "interpretation": [
        "All measurements use ROS simulation time. Exact integer source header stamps join the population.",
        "The gap between independent ROS1 observer and ROS2 node callback is a proxy, not bridge internal latency.",
        "ROS1 and ROS2 /clock delivery phase and quantization can bias the gap; negative gaps are explicit.",
        "ROS2 timing contains processed bundles only; unmatched callbacks are not proof of DDS loss.",
        "Common source windows may be shorter than either collection duration. Duplicate stamps use the first receipt.",
        "The extra ROS1 full-image subscription adds transport load even though AnyMsg avoids pixel deserialization."]}


def collect(args):
    import rospy  # Lazy: offline join/tests require no ROS installation or graph.
    from rosgraph_msgs.msg import Clock
    rospy.init_node("stereo_ros1_transport_observer", anonymous=True, disable_signals=False)
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("/use_sim_time must already be true; the probe never changes it")
    rows, errors, lock = [], [], threading.Lock()
    state = {"start_ns": None, "last_ns": 0, "last_clock_ns": 0, "clock_ticks": 0,
             "repeated_clock_ticks": 0, "clock_rewound": False, "capacity_exceeded": False,
             "uninitialized_callbacks": 0, "end_ns": None}
    def clock_callback(message):
        try:
            stamp = clock_stamp(message)
            with lock:
                if stamp < state["last_clock_ns"]:
                    state["clock_rewound"] = True
                if stamp == state["last_clock_ns"]:
                    state["repeated_clock_ticks"] += 1
                state["last_clock_ns"] = stamp
                state["clock_ticks"] += 1
        except Exception as error:
            with lock:
                errors.append(str(error))
    def callback(message, stream):
        try:
            with lock:
                now = rospy.Time.now().to_nsec()
                sequence, stamp, frame = serialized_header(message._buff)
                if now < state["last_ns"]:
                    state["clock_rewound"] = True
                state["last_ns"] = now
                state["end_ns"] = now
                if now <= 0:
                    state["uninitialized_callbacks"] += 1
                    return
                if state["start_ns"] is None:
                    state["start_ns"] = now
                if now - state["start_ns"] > args.duration_sim_s * 1e9:
                    return
                if len(rows) >= args.max_rows:
                    state["capacity_exceeded"] = True
                    return
                rows.append({"stream": stream, "sequence": sequence, "source_stamp_ns": stamp,
                             "frame_id": frame, "observer_sim_ns": now,
                             "clock_valid": int(not state["clock_rewound"] and 0 < stamp <= now
                                                and state["last_clock_ns"] > 0),
                             "clock_tick_count": state["clock_ticks"], "last_clock_message_ns": state["last_clock_ns"],
                             "local_minus_clock_message_ns": now - state["last_clock_ns"]})
        except Exception as error:
            with lock:
                errors.append(str(error))
    subscribers = [rospy.Subscriber("/clock", Clock, clock_callback, queue_size=10)]
    for stream, topic in (("left", args.left_topic), ("right", args.right_topic)):
        subscribers.append(rospy.Subscriber(topic, rospy.AnyMsg, callback, callback_args=stream,
                                           queue_size=1, buff_size=args.receive_buffer_bytes))
    rate = rospy.Rate(100)  # ROS simulation-time sleep; external timeout guards a frozen /clock.
    try:
        while not rospy.is_shutdown():
            with lock:
                start = state["start_ns"]
                done = start is not None and rospy.Time.now().to_nsec() - start >= args.duration_sim_s * 1e9
                invalid = bool(errors) or state["clock_rewound"] or state["capacity_exceeded"]
            if done or invalid:
                break
            rate.sleep()
    except (rospy.ROSInterruptException, rospy.ROSTimeMovedBackwardsException):
        pass
    with lock:
        start = state["start_ns"]
        end = rospy.Time.now().to_nsec()
        complete = start is not None and end - start >= args.duration_sim_s * 1e9
        summary = dict(state, completed_sim_duration=complete, errors=list(errors),
                       duration_sim_s=(end - start) / 1e9 if start is not None else None,
                       success=complete and not errors and not state["clock_rewound"] and not state["capacity_exceeded"],
                       clock_staleness="No wall-time stale estimate. Frozen /clock requires external timeout; repeated ticks counted.")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    with (args.output_dir / "ros1_observer.csv").open("w", newline="") as output:
        fields = ["stream", "sequence", "source_stamp_ns", "frame_id", "observer_sim_ns", "clock_valid",
                  "clock_tick_count", "last_clock_message_ns", "local_minus_clock_message_ns"]
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / "probe_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return 0 if summary["success"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    probe = commands.add_parser("ros1", help="Run in a separate ROS1 noetic environment with /use_sim_time")
    probe.add_argument("--left-topic", default="/camera_left/image_raw")
    probe.add_argument("--right-topic", default="/camera_right/image_raw")
    probe.add_argument("--duration-sim-s", type=float, default=10.)
    probe.add_argument("--max-rows", type=int, default=100000)
    probe.add_argument("--receive-buffer-bytes", type=int, default=8388608)
    probe.add_argument("--output-dir", type=Path, required=True)
    join = commands.add_parser("join", help="Offline exact-source-stamp population comparison")
    join.add_argument("--probe-csv", type=Path, required=True)
    join.add_argument("--timing-csv", type=Path, required=True)
    join.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "ros1":
        if not math.isfinite(args.duration_sim_s) or args.duration_sim_s <= 0 or args.max_rows <= 0 or args.receive_buffer_bytes <= 0:
            parser.error("duration and capacity must be positive and finite")
        if Path(__file__).resolve().parents[2] in args.output_dir.resolve().parents:
            parser.error("measurement output must be outside the checkout")
        if args.output_dir.exists():
            parser.error("output directory must not exist; failed runs are preserved")
        return collect(args)
    with args.probe_csv.open(newline="") as source:
        probe_rows = list(csv.DictReader(source))
    with args.timing_csv.open(newline="") as source:
        timing_rows = list(csv.DictReader(source))
    args.output.write_text(json.dumps(join_rows(probe_rows, timing_rows), indent=2, allow_nan=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
