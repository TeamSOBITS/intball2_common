#!/usr/bin/env python3
"""Offline static ISS visual-mesh ray references at archived camera poses.

This measures geometry, not Gazebo rendering. Dynamic models, alpha textures,
lighting, rasterization, and ROS execution are outside the reference.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shlex
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from depth_comparison import decode_depth, depth_metrics, resample_reference


class TriangleRaycaster:
    """Numpy BVH without rtree/Embree; unnormalized optical rays retain Z depth."""

    def __init__(self, triangles, leaf_size=16):
        self.triangles = np.asarray(triangles, dtype=np.float64)
        if self.triangles.ndim != 3 or self.triangles.shape[1:] != (3, 3) or not self.triangles.size:
            raise ValueError("triangles must have shape (N, 3, 3)")
        if not np.all(np.isfinite(self.triangles)) or leaf_size < 1:
            raise ValueError("invalid triangles or leaf size")
        self.nodes = []
        self._build(np.arange(len(self.triangles)), leaf_size)

    def _build(self, indices, leaf_size):
        triangles = self.triangles[indices]
        lower, upper = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
        index = len(self.nodes)
        self.nodes.append(None)
        if len(indices) <= leaf_size:
            self.nodes[index] = (lower, upper, None, None, indices)
        else:
            centroids = triangles.mean(axis=1)
            axis = int(np.argmax(np.ptp(centroids, axis=0)))
            ordered = indices[np.argsort(centroids[:, axis], kind="stable")]
            middle = len(ordered) // 2
            left = self._build(ordered[:middle], leaf_size)
            right = self._build(ordered[middle:], leaf_size)
            self.nodes[index] = (lower, upper, left, right, None)
        return index

    def raycast(self, origin, directions, max_depth, front_faces_only=False):
        origin = np.asarray(origin, dtype=np.float64)
        directions = np.asarray(directions, dtype=np.float64)
        if origin.shape != (3,) or directions.ndim != 2 or directions.shape[1] != 3:
            raise ValueError("invalid origin or ray directions")
        if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(directions)) or not np.isfinite(max_depth) or max_depth <= 0:
            raise ValueError("invalid ray values or depth limit")
        best = np.full(len(directions), max_depth, dtype=np.float64)
        faces = np.full(len(directions), -1, dtype=np.int64)
        parallel = np.abs(directions) < 1e-15
        inverse = np.divide(1.0, directions, out=np.zeros_like(directions), where=~parallel)
        stack = [(0, np.arange(len(directions)))]
        while stack:
            node, candidates = stack.pop()
            lower, upper, left, right, triangles = self.nodes[node]
            low = (lower - origin) * inverse[candidates]
            high = (upper - origin) * inverse[candidates]
            near, far = np.minimum(low, high), np.maximum(low, high)
            is_parallel = parallel[candidates]
            outside = (origin < lower) | (origin > upper)
            near = np.where(is_parallel, np.where(outside, np.inf, -np.inf), near)
            far = np.where(is_parallel, np.where(outside, -np.inf, np.inf), far)
            entering = np.maximum(near.max(axis=1), 0.0)
            exiting = np.minimum(far.min(axis=1), best[candidates])
            candidates = candidates[entering <= exiting]
            if not len(candidates):
                continue
            if triangles is None:
                stack.append((right, candidates))
                stack.append((left, candidates))
                continue
            vertices = self.triangles[triangles]
            edge1, edge2 = vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0]
            rays = directions[candidates]
            p = np.cross(rays[:, None, :], edge2[None, :, :])
            determinant = np.einsum("rtk,tk->rt", p, edge1)
            valid = determinant > 1e-12 if front_faces_only else np.abs(determinant) > 1e-12
            inverse_determinant = np.divide(1.0, determinant, out=np.zeros_like(determinant), where=valid)
            offset = origin - vertices[:, 0]
            u = np.einsum("rtk,tk->rt", p, offset) * inverse_determinant
            q = np.cross(offset, edge1)
            v = np.einsum("rk,tk->rt", rays, q) * inverse_determinant
            distance = np.einsum("tk,tk->t", edge2, q)[None, :] * inverse_determinant
            valid &= (u >= -1e-10) & (v >= -1e-10) & (u + v <= 1 + 1e-10)
            valid &= (distance > 1e-8) & (distance <= best[candidates, None])
            distance = np.where(valid, distance, np.inf)
            selected = np.argmin(distance, axis=1)
            depth = distance[np.arange(len(candidates)), selected]
            changed = np.isfinite(depth)
            best[candidates[changed]] = depth[changed]
            faces[candidates[changed]] = triangles[selected[changed]]
        best[faces < 0] = np.nan
        return best, faces


def visual_scene(urdf, package_root):
    from scipy.spatial.transform import Rotation
    import trimesh

    robot = ET.parse(urdf).getroot()
    link = robot.find("link[@name='iss_body']")
    if link is None:
        raise ValueError("URDF needs static visuals in iss_body")
    triangles, labels, records = [], [], []
    for visual in link.findall("visual"):
        mesh_spec = visual.find("geometry/mesh")
        if mesh_spec is None:
            raise ValueError("non-mesh visual is unsupported")
        uri = mesh_spec.attrib["filename"]
        prefix = "package://intball2_programs/"
        if not uri.startswith(prefix):
            raise ValueError("unsupported mesh URI " + uri)
        path = package_root / uri[len(prefix):]
        dae = ET.parse(path).getroot()
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        unit = float(dae.find("c:asset/c:unit", ns).attrib["meter"])
        up_axis = dae.find("c:asset/c:up_axis", ns).text
        if unit != 1.0 or up_axis != "Z_UP":
            raise ValueError("unverified Collada unit/up axis")
        mesh = trimesh.load(path, force="mesh", process=False)
        origin = visual.find("origin")
        xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        scale = np.fromstring(mesh_spec.attrib.get("scale", "1 1 1"), sep=" ")
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        vertices = (mesh.vertices * scale) @ transform[:3, :3].T + xyz
        label = len(records)
        triangles.append(vertices[mesh.faces])
        labels.append(np.full(len(mesh.faces), label, dtype=np.int64))
        records.append({"name": visual.attrib.get("name"), "uri": uri, "path": str(path),
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "triangles": len(mesh.faces), "origin_xyz": xyz.tolist(), "origin_rpy": rpy.tolist(),
                        "scale": scale.tolist(), "collada_unit_m": unit, "up_axis": up_axis,
                        "collada_node_transforms": "applied by trimesh.load(force='mesh')",
                        "transparent_colors": [element.text for element in dae.findall(".//c:transparent/c:color", ns)],
                        "transparency_factors": [element.text for element in dae.findall(".//c:transparency/c:float", ns)]})
    if not triangles:
        raise ValueError("no visual triangles")
    return TriangleRaycaster(np.concatenate(triangles)), np.concatenate(labels), records


def render(caster, calibration, pose, min_depth, max_depth, front_faces_only=False, max_euclidean_range=None):
    k = np.asarray(calibration["k"], dtype=np.float64).reshape(3, 3)
    transform = np.asarray(pose["world_to_optical"], dtype=np.float64)
    if (not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0 or
            not np.allclose(k[[0, 1, 2, 2, 2], [1, 0, 0, 1, 2]], [0, 0, 0, 0, 1])):
        raise ValueError("calibration needs finite zero-skew pinhole intrinsics")
    if not 0 < min_depth < max_depth or not np.all(np.isfinite([min_depth, max_depth])):
        raise ValueError("invalid depth range")
    if pose["parent"] != "iss_body" or transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("pose must map camera optical coordinates into iss_body")
    if not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-7) or not np.allclose(transform[3], [0, 0, 0, 1]):
        raise ValueError("pose must be a rigid homogeneous transform")
    u, v = np.meshgrid(np.arange(calibration["width"]), np.arange(calibration["height"]))
    rays = np.stack(((u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones_like(u)), axis=-1)
    directions = rays.reshape(-1, 3) @ transform[:3, :3].T
    depths, faces = caster.raycast(transform[:3, 3], directions, max_depth, front_faces_only)
    depths[depths < min_depth] = np.nan
    if max_euclidean_range is not None:
        depths[depths * np.linalg.norm(rays.reshape(-1, 3), axis=1) > max_euclidean_range] = np.nan
    return depths.reshape(u.shape).astype(np.float32), faces.reshape(u.shape)


def archived_depth(archive, metadata, name):
    image = metadata["images"][name]
    return decode_depth(SimpleNamespace(**image, data=archive[name + "_bytes"]))


def difference(left, right, mask):
    values = left[mask].astype(np.float64) - right[mask].astype(np.float64)
    return {"count": len(values), "mean_m": float(values.mean()) if len(values) else None,
            "median_m": float(np.median(values)) if len(values) else None,
            "absolute_p95_m": float(np.percentile(np.abs(values), 95)) if len(values) else None}


def plot_reference(stereo, mesh_left, virtual, mesh_ref_on_left, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 3, figsize=(12, 8), constrained_layout=True)
    panels = [(stereo, "Stereo Z depth", 0.25, 3, "viridis"),
              (mesh_left, "Static ISS visual mesh Z depth", 0.25, 3, "viridis"),
              (virtual, "Virtual voxel Z depth", 0.25, 3, "viridis"),
              (stereo - mesh_left, "Stereo minus mesh (left source pose)", -0.3, 0.3, "RdBu_r"),
              (mesh_ref_on_left - virtual, "Mesh minus virtual (reference pose)", -0.15, 0.15, "RdBu_r"),
              (mesh_left - mesh_ref_on_left, "Mesh source-pose minus ref-pose", -0.002, 0.002, "RdBu_r")]
    for axis, (values, title, lower, upper, palette) in zip(axes.flat, panels):
        display = axis.imshow(values, vmin=lower, vmax=upper, cmap=palette, interpolation="nearest")
        axis.set_title(title)
        axis.set_xlabel("u [pixel]")
        axis.set_ylabel("v [pixel]")
        figure.colorbar(display, ax=axis, label="m")
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main(argv=None):
    from scipy.spatial.transform import Rotation

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--iss-urdf", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--precision-dir", type=Path)
    parser.add_argument("--precision-variant", default="d2_b5")
    parser.add_argument("--min-depth-m", type=float, default=0.25)
    parser.add_argument("--max-depth-m", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.output_dir.resolve().is_relative_to(Path(__file__).resolve().parents[2]):
        parser.error("outputs must stay outside checkout")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    caster, labels, models = visual_scene(args.iss_urdf, args.package_root)
    results, skipped = [], []
    for path in sorted((args.capture / "input_samples").glob("*.npz")):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(archive["metadata_json_utf8"].tobytes())
            if not metadata["usable_replay"]:
                skipped.append(path.name)
                continue
            stereo = archived_depth(archive, metadata, "stereo_depth")
            reference = archived_depth(archive, metadata, "reference_depth")
            stereo_info, reference_info = metadata["camera_info"]["stereo"], metadata["camera_info"]["reference"]
            left_pose, ref_pose = metadata["poses"]["left"], metadata["poses"]["reference"]
            mesh_left, faces_left = render(caster, stereo_info, left_pose, args.min_depth_m, args.max_depth_m)
            mesh_ref, faces_ref = render(caster, reference_info, ref_pose, args.min_depth_m, args.max_depth_m,
                                          max_euclidean_range=args.max_depth_m)
            front_left, _ = render(caster, stereo_info, left_pose, args.min_depth_m, args.max_depth_m, True)
            reference_on_left = resample_reference(reference, reference_info["k"], stereo_info["k"], stereo.shape)
            mesh_ref_on_left = resample_reference(mesh_ref, reference_info["k"], stereo_info["k"], stereo.shape)
            common = np.isfinite(stereo) & np.isfinite(reference_on_left) & np.isfinite(mesh_left) & np.isfinite(mesh_ref_on_left)
            front_common = np.isfinite(front_left) & np.isfinite(mesh_left)
            left_transform, ref_transform = np.array(left_pose["world_to_optical"]), np.array(ref_pose["world_to_optical"])
            pose_translation = float(np.linalg.norm(left_transform[:3, 3] - ref_transform[:3, 3]))
            pose_angle = float(np.degrees(Rotation.from_matrix(left_transform[:3, :3].T @ ref_transform[:3, :3]).magnitude()))
            valid_faces = faces_left[np.isfinite(mesh_left) & (faces_left >= 0)]
            model_hits = {model["name"]: int(np.count_nonzero(labels[valid_faces] == index)) for index, model in enumerate(models)}
            result = {"archive": path.name,
                      "archive_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                      "capture_metadata": metadata, "source_stamp_ns": metadata["source_stamp_ns"],
                      "reference_pose_stamp_ns": metadata["reference_pose_stamp_ns"],
                      "source_to_reference_stamp_difference_sim_s": (metadata["reference_pose_stamp_ns"] - metadata["source_stamp_ns"]) / 1e9,
                      "pose_translation_difference_m": pose_translation, "pose_rotation_difference_deg": pose_angle,
                      "left_pose_mode": left_pose["mode"], "reference_pose_mode": ref_pose["mode"],
                      "range_contract": {"mesh_source_max_optical_z_m": args.max_depth_m,
                                         "mesh_reference_max_euclidean_range_m": args.max_depth_m,
                                         "min_optical_z_m": args.min_depth_m},
                      "model_hit_pixels": model_hits,
                      "stereo_minus_mesh": depth_metrics(stereo, mesh_left, args.min_depth_m, args.max_depth_m),
                      "mesh_minus_virtual_same_reference_pose": depth_metrics(mesh_ref, reference, args.min_depth_m, args.max_depth_m),
                      "stereo_minus_virtual": depth_metrics(stereo, reference_on_left, args.min_depth_m, args.max_depth_m),
                      "common_pixel_decomposition": {
                          "stereo_minus_virtual": difference(stereo, reference_on_left, common),
                          "stereo_minus_mesh_left": difference(stereo, mesh_left, common),
                          "mesh_left_minus_mesh_reference_pose": difference(mesh_left, mesh_ref_on_left, common),
                          "mesh_reference_pose_minus_virtual": difference(mesh_ref_on_left, reference_on_left, common)},
                      "frontface_sensitivity": {"two_sided_valid": int(np.isfinite(mesh_left).sum()),
                                                "front_only_valid": int(np.isfinite(front_left).sum()),
                                                "front_minus_two_sided": difference(front_left, mesh_left, front_common)}}
            if args.precision_dir is not None:
                candidate_dir = args.precision_dir / path.stem / args.precision_variant
                lines = (candidate_dir / "metadata.txt").read_text().splitlines()
                width, height, candidate_stamp, candidate_frame = shlex.split(lines[0])
                candidate_stamp, width, height = int(candidate_stamp), int(width), int(height)
                if candidate_stamp != metadata["source_stamp_ns"] or candidate_frame != stereo_info["frame_id"]:
                    raise ValueError("candidate stamp/frame differs from captured source")
                candidate_k = np.fromstring(lines[1], sep=" ")
                candidate = np.fromfile(candidate_dir / "depth.bin", dtype=np.float32).reshape(height, width)
                sampled = resample_reference(candidate, candidate_k, stereo_info["k"], stereo.shape)
                candidate_info = {"width": width, "height": height, "k": candidate_k}
                exact_candidate_mesh, _ = render(caster, candidate_info, left_pose, args.min_depth_m, args.max_depth_m)
                sampled_candidate_mesh = resample_reference(exact_candidate_mesh, candidate_k, stereo_info["k"], stereo.shape)
                common_candidates = np.isfinite(stereo) & np.isfinite(sampled) & np.isfinite(mesh_left)
                result["precision_variant"] = {
                    "metadata_sha256": hashlib.sha256((candidate_dir / "metadata.txt").read_bytes()).hexdigest(),
                    "depth_sha256": hashlib.sha256((candidate_dir / "depth.bin").read_bytes()).hexdigest(),
                    "name": args.precision_variant, "native_shape": [height, width],
                    "evaluation": "Nearest candidate samples on captured 200-grid; same mesh200 reference and common-valid pixels",
                    "candidate_minus_mesh_200": depth_metrics(sampled, mesh_left, args.min_depth_m, args.max_depth_m),
                    "candidate_minus_exact_native_mesh": depth_metrics(candidate, exact_candidate_mesh, args.min_depth_m, args.max_depth_m),
                    "common_baseline_minus_mesh": depth_metrics(np.where(common_candidates, stereo, np.nan), np.where(common_candidates, mesh_left, np.nan), args.min_depth_m, args.max_depth_m),
                    "common_candidate_minus_mesh": depth_metrics(np.where(common_candidates, sampled, np.nan), np.where(common_candidates, mesh_left, np.nan), args.min_depth_m, args.max_depth_m),
                    "nearest_ray_geometry_shift": difference(sampled_candidate_mesh, mesh_left, np.isfinite(sampled_candidate_mesh) & np.isfinite(mesh_left))}
                np.savez_compressed(args.output_dir / (path.stem + "_candidate.npz"), candidate_depth_m=sampled,
                                    candidate_exact_native_mesh_depth_m=exact_candidate_mesh,
                                    candidate_sampled_mesh_depth_m=sampled_candidate_mesh, common_candidate_mask=common_candidates)
            np.savez_compressed(args.output_dir / path.name, mesh_left_depth_m=mesh_left, mesh_reference_pose_depth_m=mesh_ref,
                                mesh_front_only_depth_m=front_left, stereo_depth_m=stereo, virtual_depth_m=reference_on_left,
                                mesh_reference_pose_on_left_depth_m=mesh_ref_on_left,
                                model_face_index=faces_left, common_mask=common)
            if args.plot:
                plot_reference(stereo, mesh_left, reference_on_left, mesh_ref_on_left,
                               args.output_dir / (path.stem + "_geometry.png"))
            results.append(result)
            print(json.dumps({"archive": path.name, "stereo_mesh_median_m": result["stereo_minus_mesh"]["signed_median_m"],
                              "mesh_virtual_median_m": result["mesh_minus_virtual_same_reference_pose"]["signed_median_m"]}), flush=True)
    summary = {"success": bool(results), "scope": "Static iss_body URDF visual meshes only; optical-axis Z depth, same pixel rays and archived source/reference poses",
               "limitations": ["dynamic models and their contemporaneous poses are not archived", "materials/textures/alpha and rasterization are not modeled", "two-sided intersections are primary; front-only sensitivity is reported", "Gazebo loaded model equivalence needs independent confirmation; source URDF is the hypothesis", "TF may interpolate at requested pose stamp; reference and stereo source stamps differ"],
               "arguments": {name: str(value) if isinstance(value, Path) else value for name, value in vars(args).items()},
               "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "urdf_sha256": hashlib.sha256(args.iss_urdf.read_bytes()).hexdigest(), "models": models,
               "results": results, "skipped_unusable_archives": skipped}
    (args.output_dir / "mesh_reference_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return 0 if results else 1


if __name__ == "__main__":
    raise SystemExit(main())
