#!/usr/bin/env python3
"""Summarize saved SIP native/adapter clocks and exact-stamp observer joins."""

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path


def distribution(values):
    values = sorted(values)
    def percentile(fraction):
        position = (len(values) - 1) * fraction
        low = int(position)
        high = min(low + 1, len(values) - 1)
        return values[low] + (values[high] - values[low]) * (position - low)
    return {"count": len(values), "min_s": values[0] if values else None,
            "max_s": values[-1] if values else None,
            "p50_s": percentile(.5) if values else None,
            "p90_s": percentile(.9) if values else None,
            "p95_s": percentile(.95) if values else None}


def summarize_rows(rows):
    result = {"row_count": len(rows)}
    for prefix in ("native_timing", "timing"):
        result[prefix + "_valid_count"] = sum(row[prefix + "_valid"] == "True" for row in rows)
        result[prefix + "_invalid_reasons"] = dict(Counter(
            row[prefix + "_invalid_reason"] or "unspecified"
            for row in rows if row[prefix + "_valid"] != "True"))
    result["conversion_success_count"] = sum(row["conversion_success"] == "True" for row in rows)
    result["conversion_rejection_reasons"] = dict(Counter(
        row["conversion_rejection_reason"] or "unspecified"
        for row in rows if row["conversion_success"] != "True"))
    for key in ("source_to_disparity_receive_sim_s", "disparity_receive_to_conversion_start_sim_s",
                "adapter_conversion_sim_s", "adapter_ready_to_publish_sim_s", "source_to_adapter_publish_sim_s"):
        validity = "native_timing_valid" if key == "source_to_disparity_receive_sim_s" else "timing_valid"
        selected = [row for row in rows if row[validity] == "True"]
        values = []
        excluded = 0
        for row in selected:
            try:
                value = float(row[key])
                if not math.isfinite(value) or value < 0:
                    raise ValueError("invalid duration")
                values.append(value)
            except (ValueError, KeyError):
                excluded += 1
        result[key] = {**distribution(values), "excluded_missing_negative_nonfinite_count": excluded}
    return result


def summarize(adapter_dir, quality_dir, adapter_phase_clock_status="unverified"):
    adapter_path = adapter_dir / "summary.json"
    native_path = adapter_dir / "native_adapter_timing.csv"
    quality_path = quality_dir / "summary.json"
    timing_path = quality_dir / "timing.csv"
    adapter = json.loads(adapter_path.read_text())
    quality = json.loads(quality_path.read_text())
    with native_path.open() as stream:
        rows = list(csv.DictReader(stream))
    with timing_path.open() as stream:
        observer = list(csv.DictReader(stream))
    start, end = quality["start_sim_ns"], quality["end_sim_ns"]
    window_rows = [row for row in rows if start <= int(row["disparity_receive_sim_ns"]) <= end]
    native_stamps = {int(row["source_stamp_ns"]) for row in rows}
    observer_stamps = {int(row["source_stamp_ns"]) for row in observer}
    quality_stamps = {int(row["stamp_ns"]) for row in quality["quality_samples"]}
    shared = native_stamps & observer_stamps
    joined = [row for row in rows if int(row["source_stamp_ns"]) in shared]
    files = [adapter_path, native_path, quality_path, timing_path]
    return {"clock": "ROS simulation time only; native publish time is unknown",
            "adapter_phase_clock_status": adapter_phase_clock_status,
            "adapter_phase_interpretation": (
                "Unavailable: callback blocked simulation clock subscription updates; recorded zero durations and adapter publish clock are not valid elapsed measurements"
                if adapter_phase_clock_status == "unavailable_clock_updates_during_callback" else
                "Clock update availability must be independently verified; numeric zeros alone do not establish elapsed duration"),
            "restrictions": ["Native receive age includes SIP processing and delivery, not internal compute duration",
                             "Adapter publish clock follows depth+CameraInfo publish completion",
                             "Quality window native rows are selected by disparity receipt time; exact-source joins are separate",
                             "Failed quality captures remain in this report; absent depth has no measured error",
                             "Carrier downsample changes adapter output/payload only, not native SIP resolution",
                             "No cause of missing observer depth is established by this report"],
            "inputs": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
            "adapter_capture": {"complete": adapter["complete"], "stop_reason": adapter["stop_reason"],
                                "counts": adapter["counts"], "arguments": adapter["arguments"],
                                "runtime_exception": adapter.get("runtime_exception")},
            "native_whole_capture": summarize_rows(rows),
            "quality_capture": {"success": quality["success"], "failure": quality.get("failure"),
                                "capture_complete": quality.get("capture_complete"),
                                "start_sim_ns": start, "end_sim_ns": end,
                                "duration_sim_s": (end - start) / 1e9,
                                "timing_count": len(observer), "quality_count": len(quality["quality_samples"]),
                                "input_stream_stamp_counts": quality.get("input_stream_stamp_counts"),
                                "observer_source_to_output_sim_s": quality.get("source_to_output_sim_s")},
            "native_received_in_quality_window": summarize_rows(window_rows),
            "same_source_stamp_observer_join": {"unique_native_stamps": len(native_stamps),
                                                "unique_observer_stamps": len(observer_stamps),
                                                "joined_unique_stamps": len(shared),
                                                "quality_joined_unique_stamps": len(native_stamps & quality_stamps),
                                                "observer_without_native_stamps": sorted(observer_stamps - native_stamps),
                                                "native_adapter_metrics": summarize_rows(joined)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-dir", type=Path, required=True)
    parser.add_argument("--quality-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--adapter-phase-clock-status", choices=("unverified", "available", "unavailable_clock_updates_during_callback"), default="unverified")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; preserve earlier results")
    result = summarize(args.adapter_dir, args.quality_dir, args.adapter_phase_clock_status)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
