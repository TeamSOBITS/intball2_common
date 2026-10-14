<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

# intball2_stereo

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#実行方法">実行方法</a>
      <ul>
        <li><a href="#ノードの起動">ノードの起動</a></li>
        <li><a href="#トピック">トピック</a></li>
      </ul>
    </li>
  </ol>
</details>

## 概要

Int-Ball2 の左右カメラ画像からステレオ視差を求め，深度画像（任意で点群）を配信するパッケージです．

<p align="center">
  <img src="images/stereo_rgb_depth.png" alt="Gazebo の同一フレームの左 RGB 画像とステレオ深度画像．深度（32FC1）は白黒表示で，白は近距離，黒は遠距離，灰色は無効画素．" width="960">
</p>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 実行方法

### ノードの起動

`stereo.launch.py` を起動します．

```sh
ros2 launch intball2_stereo stereo.launch.py
```

- 精度優先設定
  - 細かい画像で深度を補正するため，処理負荷が増えます．
  ```sh
  ros2 launch intball2_stereo stereo.launch.py precision:=true
  ```
- 点群について
  - デフォルトでは配信しません．`points.publish_points: true` で配信します．
  - launch起動中でも次のコマンドで有効にできます．
  ```sh
  ros2 param set /stereo_node points.publish_points true
  ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

#### パラメータ一覧

以下は launch が読み込む YAML の値です．「精度優先」が「同じ」の項目は通常設定と共通です．

**画像処理・深度・点群**

| パラメータ | 通常 | 精度優先 | 用途・指定できる値 |
| --- | --- | --- | --- |
| `camera.calibration_mode` | `gazebo_negative_5cm` | 同じ | 校正の扱い．既存 Gazebo 用は `gazebo_negative_5cm`，通常の CameraInfo 用は `camera_info` |
| `camera.input_geometry` | `already_rectified` | 同じ | 入力画像の状態．補正済みは `already_rectified`，未補正は `raw` |
| `processing.downsample` | `4` | 同じ | 画像の縮小率．`1`，`2`，`4`．800×800 の入力は既定で 200×200 に縮小 |
| `processing.opencv_threads` | `1` | `4` | OpenCV のスレッド数．`1`〜`64` |
| `processing.roi_left_px` | `0` | 同じ | 処理範囲から除く左端の幅（出力画像の画素単位） |
| `processing.roi_right_px` | `0` | 同じ | 処理範囲から除く右端の幅（出力画像の画素単位） |
| `processing.roi_top_px` | `0` | 同じ | 処理範囲から除く上端の幅（出力画像の画素単位） |
| `processing.roi_bottom_px` | `0` | 同じ | 処理範囲から除く下端の幅（出力画像の画素単位） |
| `depth.min_depth_m` | `0.25` | 同じ | 出力する深度の下限（m） |
| `depth.max_depth_m` | `3.0` | 同じ | 出力する深度の上限（m） |
| `points.publish_points` | `false` | 同じ | `true` で `/stereo/points` に点群を配信 |
| `points.layout` | `organized` | 同じ | `organized` は画像と同じ行列，`compact` は有効点だけを出力 |
| `points.stride` | `1` | 同じ | 点群の間引き間隔．`compact` 用．`organized` では `1` |

ROI の各値は補正済み画像用です．範囲外の深度は無効値（NaN）になり，出力画像のサイズは変わりません．両 YAML は Gazebo の未補正 CameraInfo 用です．`gazebo_negative_5cm` と `*_fixed` の CameraInfo は併用しないでください．実機用には校正と画像の状態に合わせた設定が必要です．

**左右画像の照合**

左右画像の同じ場所を探すための設定です．画素単位の値は縮小後の画像を基準にします．縮小率を変える場合は，必要な測距範囲に合わせて視差の探索範囲も調整してください．

| パラメータ | 通常・精度優先 | 用途・指定できる値 |
| --- | --- | --- |
| `matcher.min_disparity_px` | `-25` | 視差の探索開始値（画素） |
| `matcher.num_disparities_px` | `32` | 視差の探索幅（画素）．正の16の倍数 |
| `matcher.disparity_units` | `processed_pixels` | 視差の単位．`processed_pixels` 固定 |
| `matcher.block_size_px` | `5` | 照合に使う窓の幅（画素）．`3`〜`255` の奇数 |
| `matcher.uniqueness_ratio` | `10` | 照合候補を採用するための差のしきい値（%）．`0`〜`100` |
| `matcher.speckle_window_size_px` | `100` | 除去する孤立領域の最大面積（画素数）．`0` で無効 |
| `matcher.speckle_range_px` | `2` | 同じ孤立領域とみなす視差の差（画素） |
| `matcher.disp12_max_diff_px` | `1` | 左右の照合結果に許す差（画素）．`-1` で無効 |

**精度補正**

`refinement.enabled=true` のときに使う設定です．補正の画像縮小率・窓幅・探索幅は，まず精度優先 YAML の組合せを使ってください．

| パラメータ | 通常 | 精度優先 | 用途・指定できる値 |
| --- | --- | --- | --- |
| `refinement.enabled` | `false` | `true` | 深度の追加補正を有効にする |
| `refinement.downsample` | `2` | `1` | 補正用画像の縮小率．`1`，`2`，`4`．照合用より粗い画像にはできない |
| `refinement.window_size_px` | `9` | `17` | 補正用の窓幅（補正画像の画素）．`3`〜`31` の奇数 |
| `refinement.search_radius_px` | `2` | `3` | 補正の探索半径（補正画像の画素）．`1`〜`8` |
| `refinement.min_texture_variance` | `1.0` | 同じ | 補正に必要な画像の明暗変化の下限．正の値 |
| `refinement.min_correlation` | `0.7` | 同じ | 補正結果を採用する画像類似度の下限．`0`〜`1` |

**同期・通信・診断**

| パラメータ | 通常・精度優先 | 用途・指定できる値 |
| --- | --- | --- |
| `sync.queue_size` | `5` | 入力を同期するために保持する件数．`1`〜`1000` |
| `sync.max_interval_s` | `0.02` | 4入力の時刻に許す最大差（秒）．`0`〜`1` |
| `qos.depth` | `5` | 通信キューの保持件数．`1`〜`1000` |
| `diagnostics.publish_interval_s` | `1.0` | 診断の配信間隔（実時間の秒）．`0.01`〜`60` |

`topics.*`，`sync.*`，`qos.depth`，`diagnostics.publish_interval_s`，`processing.opencv_threads` を変更するにはノードの再起動が必要です．他の表中パラメータは `ros2 param set` でも変更できます．関連する値をまとめて変える場合は YAML を編集して起動し直してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### トピック

`topics.*` の値は絶対トピック名（`/` で始まる名前）で指定します．

| パラメータ | 方向 | 既定トピック | 型 | 内容 |
| --- | --- | --- | --- | --- |
| `topics.left_image` | 入力 | `/camera_left/image_raw` | `sensor_msgs/Image` | 左画像 |
| `topics.right_image` | 入力 | `/camera_right/image_raw` | `sensor_msgs/Image` | 右画像 |
| `topics.left_camera_info` | 入力 | `/camera_left/camera_info` | `sensor_msgs/CameraInfo` | 左カメラの校正情報 |
| `topics.right_camera_info` | 入力 | `/camera_right/camera_info` | `sensor_msgs/CameraInfo` | 右カメラの校正情報 |
| `topics.depth` | 出力 | `/stereo/depth` | `sensor_msgs/Image` | 深度画像（m） |
| `topics.camera_info` | 出力 | `/stereo/camera_info` | `sensor_msgs/CameraInfo` | 出力画像に対応する校正情報 |
| `topics.points` | 出力 | `/stereo/points` | `sensor_msgs/PointCloud2` | `points.publish_points=true` のときに配信する XYZ 点群 |
| `topics.timing` | 出力 | `/stereo/timing` | `diagnostic_msgs/DiagnosticArray` | 画像取得から深度配信直前までのシム時間の遅延 |
| `topics.diagnostics` | 出力 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 処理状態とエラー |

画像と CameraInfo は毎フレーム配信し，4入力の時刻差を `sync.max_interval_s` 以内にしてください．時刻は単調増加が必要です．シムの時計を巻き戻した場合はノードを起動し直してください．通常の `camera_info` モードでは，画像と CameraInfo の frame を同じ optical frame に揃えてください．

入力と，診断以外の出力は best-effort / volatile の QoS を使います．深度などを受け取る側も best-effort に合わせてください．`/diagnostics` は reliable / volatile です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>
