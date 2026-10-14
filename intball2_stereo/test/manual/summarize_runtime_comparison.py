#!/usr/bin/env python3
"""Summarize collector runs without dropping failures.

Schema: runs retain success/failure and clock verification. timing contains seconds
p50/p95 and valid/sample/excluded counts; absent measurements stay null. quality is
the unweighted median of finite per-frame metrics, not pooled-pixel error. Diagnostic
counter differences cover their captured first/last snapshots, not the full run.
"""
import argparse
import json
import math
from pathlib import Path
from statistics import median

TIMING = (
    "source_to_output_sim_s", "source_to_publish_sim_s", "source_to_depth_ready_sim_s",
    "processing_sim_s", "source_to_left_callback_sim_s", "left_callback_to_bundle_sim_s",
    "bundle_to_worker_start_sim_s", "worker_setup_sim_s", "preprocessing_sim_s",
    "matching_sim_s", "depth_conversion_sim_s", "refinement_sim_s",
    "core_output_metadata_sim_s", "output_copy_sim_s", "depth_ready_to_publish_sim_s",
    "reference_header_to_output_sim_s",
)
QUALITY = ("stereo_valid_fraction", "signed_mean_m", "signed_median_m", "absolute_p90_m",
           "absolute_p95_m", "rmse_m", "too_near_fraction", "too_far_fraction")


def summarize(directory):
    source = directory / "summary.json"
    try:
        data = json.loads(source.read_text())
    except (OSError, ValueError) as error:
        return {"run": str(directory), "success": False, "failure": str(error),
                "summary_available": False, "clock_verified": False}
    clock_verified = data.get("clock") == "ROS use_sim_time"
    timing = {}
    for key in TIMING:
        values = data.get(key, {}) if clock_verified else {}
        timing[key] = {field: values.get(field) for field in (
            "sample_count", "valid_count", "excluded_negative_count", "excluded_nonfinite_count",
            "p50_s", "p95_s")}
    samples = data.get("quality_samples", [])
    quality = {}
    for key in QUALITY:
        values = [sample.get("metrics", {}).get(key) for sample in samples]
        finite = [value for value in values if isinstance(value, (int, float)) and math.isfinite(value)]
        quality[key] = {"median": median(finite) if finite else None, "finite_sample_count": len(finite),
                        "missing_sample_count": len(values) - len(finite)}
    diagnostics = data.get("node_diagnostics_window", {})
    return {"run": str(directory), "summary_available": True,
            "success": data.get("success"), "failure": data.get("failure"),
            "clock": data.get("clock"), "clock_verified": clock_verified,
            "start_sim_ns": data.get("start_sim_ns"), "end_sim_ns": data.get("end_sim_ns"),
            "timing_count": data.get("timing_count"), "quality_sample_count": len(samples),
            "output_hz_sim": data.get("output_hz_sim") if clock_verified else None,
            "pose_max_translation_m": data.get("pose_max_translation_m"),
            "pose_max_angle_deg": data.get("pose_max_angle_deg"),
            "timing": timing, "quality_frame_metric_medians": quality,
            "node_diagnostics_window": {key: diagnostics.get(key) for key in (
                "missing", "measurement", "first_diag_sim_ns", "last_diag_sim_ns",
                "covered_duration_sim_s", "shorter_than_capture_window", "counter_delta",
                "config_revision_changed")},
            "reference_header_stamp_meaning": data.get("reference_timing_metadata", {}).get("header_stamp_meaning")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    runs = [summarize(directory) for directory in args.run_dirs]
    report = {"schema_version": 1, "clock_requirement": "ROS use_sim_time",
              "population": "Every requested run, including failures and missing summaries; no pooled aggregation",
              "quality_definition": "Unweighted median of finite per-frame metrics; missing values counted",
              "diagnostics_definition": "Captured diagnostic snapshot interval only; not full capture duration",
              "reference_limit": "Virtual header uses body TF stamp, not image acquisition; depth is approximate box geometry",
              "runs": runs}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    for run in runs:
        timing = run.get("timing", {})
        latency = timing.get("source_to_output_sim_s", {})
        processing = timing.get("processing_sim_s", {})
        quality = run.get("quality_frame_metric_medians", {}).get("rmse_m", {})
        print(f"{Path(run['run']).name}: success={run.get('success')} failure={run.get('failure')} "
              f"n={run.get('timing_count')} source_p50/p95_s={latency.get('p50_s')}/{latency.get('p95_s')} "
              f"processing_p50/p95_s={processing.get('p50_s')}/{processing.get('p95_s')} "
              f"frame_median_rmse_m={quality.get('median')}")


if __name__ == "__main__":
    main()
