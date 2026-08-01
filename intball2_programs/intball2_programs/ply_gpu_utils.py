"""
ply_gpu_utils.py
================
点群の座標変換・色計算をCPU(NumPy)またはGPU(CuPy/CUDA)で切り替えて実行するための
ユーティリティモジュール。

【設計方針】
- CuPyはNumPyとほぼ同一のAPIを持つGPU計算ライブラリ。
  `import cupy as cp` した上で、NumPyの`np.xxx`を`cp.xxx`に置き換えるだけで
  同じ計算ロジックをGPU上で実行できる。
- CuPyが未インストール、またはCUDA対応GPUが無い環境でもノード自体は起動できるよう、
  importはtry/exceptで保護し、is_cuda_available()で安全に判定する。
- 既存のply_publisher.py / ply_color_utils.py の計算式（数式）は一切変更せず、
  「同じ式をどちらのデバイスで計算するか」だけを切り替える。
"""

import numpy as np

try:
    import cupy as cp
    _CUPY_IMPORT_ERROR = None
except ImportError as e:
    cp = None
    _CUPY_IMPORT_ERROR = e


def is_cuda_available() -> bool:
    """CuPyがインストールされており、かつCUDA対応GPUが少なくとも1台認識できるかを判定する。

    Returns:
        bool: 使用可能なら True。CuPy未インストール／GPUドライバ不整合／
              GPUが存在しない場合は False（例外は投げない）。
    """
    if cp is None:
        return False
    try:
        return cp.cuda.runtime.getDeviceCount() > 0
    except Exception:
        # ドライバ未インストール等、実行時エラーも「利用不可」として扱う
        return False


def resolve_device(requested_device: str, logger=None) -> str:
    """ユーザーが要求したデバイス名を検証し、実際に使用するデバイス名を返す。

    Args:
        requested_device: 'cpu' または 'cuda'（大小文字は無視）。
        logger: rclpyのロガー（node.get_logger()）。Noneの場合はprintで代替。

    Returns:
        str: 'cpu' または 'cuda'。
             'cuda'が要求されたが利用できない場合は警告を出した上で 'cpu' を返す
             （ノードがクラッシュしないようにするためのフォールバック）。
    """
    requested = (requested_device or 'cpu').strip().lower()

    if requested not in ('cpu', 'cuda'):
        msg = (f"processing_device='{requested_device}' は不正な値です。"
               "'cpu' として扱います。('cpu' または 'cuda' を指定してください)")
        _log_warn(msg, logger)
        return 'cpu'

    if requested == 'cuda' and not is_cuda_available():
        reason = f'(詳細: {_CUPY_IMPORT_ERROR})' if _CUPY_IMPORT_ERROR else '(GPUが検出されませんでした)'
        msg = ('processing_device=cuda が指定されましたが、CuPyまたはCUDA対応GPUが '
               f'利用できないため CPU 処理にフォールバックします。{reason}')
        _log_warn(msg, logger)
        return 'cpu'

    return requested


def _log_warn(msg: str, logger):
    if logger is not None:
        logger.warn(msg)
    else:
        print(f'[WARN] {msg}')


def transform_points(points: np.ndarray, R: np.ndarray, shift: np.ndarray, device: str = 'cpu') -> np.ndarray:
    """点群座標に回転行列を適用し、平行移動を加える。

    計算式:
        points_out = points @ R.T + shift

    これは ply_publisher.py の元コードにあった
        points = np.dot(points, R.T)
        points[:, 0] += shift_x  ...
    と全く同じ計算を1つにまとめたもの（式の意味・結果は変更なし）。

    Args:
        points: (N, 3) 変換前の点群座標（NumPy配列）。
        R: (3, 3) 合成済み回転行列（R_z @ R_y @ R_x）。
        shift: (3,) 平行移動ベクトル [shift_x, shift_y, shift_z]。
        device: 'cpu' または 'cuda'。

    Returns:
        (N, 3) 変換後の点群座標（常にNumPy配列。GPU計算時はホストへ転送済み）。
    """
    if device == 'cuda' and cp is not None:
        pts_gpu = cp.asarray(points, dtype=cp.float64)
        R_gpu = cp.asarray(R, dtype=cp.float64)
        shift_gpu = cp.asarray(shift, dtype=cp.float64)
        out_gpu = cp.dot(pts_gpu, R_gpu.T) + shift_gpu
        return cp.asnumpy(out_gpu)

    return np.dot(points, R.T) + shift


def pack_rgb_float32(colors: np.ndarray, device: str = 'cpu') -> np.ndarray:
    """0.0〜1.0のRGB配列(N,3)を、ROS PointCloud2の'rgb'フィールド用に
    1個のfloat32へビットパックした配列(N,)へ変換する。

    計算式（元のply_publisher.pyの処理と同一）:
        1. colors_uint8 = trunc(colors * 255)   ※小数点以下切り捨て、0〜255相当のuint32
        2. rgb_packed   = (R << 16) | (G << 8) | B   （3チャンネルを1つの32bit整数に詰める）
        3. rgb_packed のビットパターンをそのまま float32 として再解釈(view)する
           （数値変換ではなく、メモリ上の32bitパターンをfloatとして読み替えるだけ。
             ROSの sensor_msgs/PointField.FLOAT32 型で'rgb'を送るための標準的な手法）

    Args:
        colors: (N, 3) 0.0〜1.0のRGB値。
        device: 'cpu' または 'cuda'。

    Returns:
        (N,) float32配列（ビットパックされたRGB）。常にNumPy配列で返す。
    """
    if device == 'cuda' and cp is not None:
        colors_gpu = cp.asarray(colors)
        colors_uint32_gpu = (colors_gpu * 255).astype(cp.uint32)
        rgb_packed_gpu = (
            (colors_uint32_gpu[:, 0] << 16)
            | (colors_uint32_gpu[:, 1] << 8)
            | colors_uint32_gpu[:, 2]
        )
        # CuPy配列のview()結果はGPU上のfloat32配列になるため、
        # PointCloud2メッセージ化のためホスト(CPU)側のNumPy配列へ転送する
        rgb_packed_host_uint32 = cp.asnumpy(rgb_packed_gpu)
        return rgb_packed_host_uint32.view(np.float32)

    colors_uint8 = (colors * 255).astype(np.uint32)
    rgb_packed = (colors_uint8[:, 0] << 16) | (colors_uint8[:, 1] << 8) | colors_uint8[:, 2]
    return rgb_packed.view(np.float32)