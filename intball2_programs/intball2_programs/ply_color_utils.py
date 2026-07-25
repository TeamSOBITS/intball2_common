import numpy as np

# PLYのプロパティ型名 → numpy dtype文字列（リトルエンディアン固定）
_PLY_TYPE_TO_NP_LE = {
    'float': '<f4', 'float32': '<f4',
    'double': '<f8', 'float64': '<f8',
    'char': '<i1', 'int8': '<i1',
    'uchar': '<u1', 'uint8': '<u1',
    'short': '<i2', 'int16': '<i2',
    'ushort': '<u2', 'uint16': '<u2',
    'int': '<i4', 'int32': '<i4',
    'uint': '<u4', 'uint32': '<u4',
}

# 球面調和関数 l=0, m=0 項（DC項）の正規化定数。
#   Y_0^0 = 1 / (2 * sqrt(pi)) ≈ 0.28209479177387814
# 3DGSの学習過程では、各点のベース色(0〜1)は
#   f_dc = (color - 0.5) / SH_C0
# としてエンコードされる。したがって復元時は逆変換
#   color = SH_C0 * f_dc + 0.5
# を行い、範囲外に出た値は [0, 1] にクリップする。
SH_C0 = 0.28209479177387814


def inspect_ply_vertex_properties(path: str):
    """PLYヘッダのみを読み、(プロパティ[(名前, 型)]のリスト, フォーマット文字列,
    頂点数) を返す。本体データは読まないため高速。"""
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError(f'PLYファイルではありません: {path}')

        fmt = None
        vertex_count = 0
        in_vertex = False
        props = []

        while True:
            line = f.readline()
            if not line:
                raise ValueError('ヘッダの途中でファイルが終了しました')
            tokens = line.strip().decode('ascii', errors='replace').split()
            if not tokens:
                continue
            head = tokens[0]
            if head == 'end_header':
                break
            if head == 'format':
                fmt = tokens[1]
            elif head == 'element':
                in_vertex = (tokens[1] == 'vertex')
                if in_vertex:
                    vertex_count = int(tokens[2])
            elif head == 'property' and in_vertex:
                if tokens[1] == 'list':
                    continue
                props.append((tokens[2], tokens[1]))  # (name, type)

        return props, fmt, vertex_count


def read_dc_colors_from_ply(path: str):
    """3DGS形式PLYの f_dc_0/f_dc_1/f_dc_2 を読み出し、SH DC項からRGB
    (0.0〜1.0 の Nx3 float32 ndarray) に変換して返す。

    f_dc_0/1/2 が存在しない場合（＝3DGS形式ではない場合）は None を返す。
    対応フォーマットは binary_little_endian のみ（3DGS PLYは通常この形式）。
    """
    props, fmt, vertex_count = inspect_ply_vertex_properties(path)
    prop_names = {name for name, _ in props}

    if not {'f_dc_0', 'f_dc_1', 'f_dc_2'} <= prop_names:
        return None  # SH形式ではない → 呼び出し側で標準RGB等を試す

    if fmt != 'binary_little_endian':
        raise ValueError(
            f'read_dc_colors_from_ply は binary_little_endian のみ対応です (fmt={fmt})')

    dtype = np.dtype([(name, _PLY_TYPE_TO_NP_LE[ptype]) for name, ptype in props])

    with open(path, 'rb') as f:
        while f.readline().strip() != b'end_header':
            pass
        buf = f.read(dtype.itemsize * vertex_count)

    if len(buf) < dtype.itemsize * vertex_count:
        raise ValueError('PLYバイナリデータが途中で終了しています')

    arr = np.frombuffer(buf, dtype=dtype, count=vertex_count)

    dc = np.stack([
        arr['f_dc_0'].astype(np.float32),
        arr['f_dc_1'].astype(np.float32),
        arr['f_dc_2'].astype(np.float32),
    ], axis=1)

    # color = SH_C0 * f_dc + 0.5 を [0, 1] にクリップ
    colors = np.clip(SH_C0 * dc + 0.5, 0.0, 1.0).astype(np.float64)
    return colors