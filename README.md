<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# intball2_common

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
  </ol>
</details>

## 概要
Int-Ball2 simulaterでロボットを制御するためのパッケージです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## セットアップ
ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件
まず，以下の環境を整えてから，次のインストール方法に進んでください．
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   cd　~/colcon_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   git clone https://github.com/TeamSOBITS/intball2_common.git
   ```
3. レポジトリの中へ移動します．
   ```sh
   cd intball2_common
   ```
4. 依存パッケージをインストールします．
    ```sh
    bash install.sh
    ```
5. パッケージをコンパイルします．
   ```sh
   cd ~/colcon_ws/
   colcon_build
   source ~/colcon_ws/devel/setup.bash
   ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## TF Tree

```sh
base
├── iss_body
│  └── dock_body
└── body
    ├── cameraF_link
    ├── cameraL_link
    ├── cameraR_link
    └── imu_link
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 実行・操作方法
はじめにInt‑Ball2 シミュレータを起動します．
次にROS2 Bridgeコンテナを起動します．

### 障害物生成方法
Gazebo上のISSに対して相対的な位置に障害物を配置できます
```sh
ros2 run intball2_programs spawn_model [引数]
```

#### 引数一覧

| 引数 | 説明 | デフォルト値 |
| --- | --- | --- |
| `-m` | モデル名 | `box_obstacle` |
| `-n` | モデルインスタンス名（省略時は自動生成） | `{model}_{uuid}` |
| `-f` | 参照フレーム | `iss_body` |
| `-s` | サイズ（メートル） | `0.45 0.25 1.7` |
| `-o` | オフセット（メートル） | `0.0 0.0 0.0` |
| `-r` | 回転角（度） | `0.0 0.0 0.0` |
| `-c` | コリジョン有効化フラグ | 無効 |

#### 使用例

```sh
# 基本的な使用例（デフォルト値で配置）
ros2 run intball2_programs spawn_model

# boxを配置
ros2 run intball2_programs spawn_model -m box

# サイズとオフセットを指定
ros2 run intball2_programs spawn_model -s 0.5 0.5 1.0 -o 0.5 0.0 0.0

# 回転を指定（Z軸中心に45度回転）
ros2 run intball2_programs spawn_model -r 0.0 0.0 45.0

# コリジョン有効で配置
ros2 run intball2_programs spawn_model -c

# 複数の引数を組み合わせて使用
ros2 run intball2_programs spawn_model -n obstacle2 -s 0.3 0.3 0.3 -o 1.0 0.5 0.0 -r 0.0 0.0 90.0 -c
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 相対移動方法
Int-Ball2を相対座標で移動させることができます
```sh
ros2 run intball2_programs move_relative [引数]
```

#### 引数一覧

| 短形式 | 長形式 | 説明 | デフォルト値 |
| --- | --- | --- | --- |
| `-x` | `--x` | 相対移動 X方向（メートル） | `0.0` |
| `-y` | `--y` | 相対移動 Y方向（メートル） | `0.0` |
| `-z` | `--z` | 相対移動 Z方向（メートル） | `0.0` |
| `-r` | `--roll` | 相対回転 ロール角（度） | `0.0` |
| `-p` | `--pitch` | 相対回転 ピッチ角（度） | `0.0` |
| `-w` | `--yaw` | 相対回転 ヨー角（度） | `0.0` |

#### 使用例

```sh
# 基本的な使用例（Z軸方向に0.5m移動）
ros2 run intball2_programs move_relative -z 0.5

# X方向とY方向に同時移動
ros2 run intball2_programs move_relative -x 0.3 -y -0.2

# 移動と回転を組み合わせ（Z軸方向に1.0m移動しながらZ軸中心に90度回転）
ros2 run intball2_programs move_relative -z 1.0 -w 90.0

# ロール角を指定
ros2 run intball2_programs move_relative -r 45.0

# 複数の動きを組み合わせ
ros2 run intball2_programs move_relative -x 0.5 -y 0.5 -z 0.5 -r 0.0 -p 0.0 -w 45.0
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## マイルストーン

現時点のバグや新規機能の依頼を確認するために[Issueページ](https://github.com/TeamSOBITS/intball_common/issues)をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/intball_common.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/intball_common/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/intball_common.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/intball_common/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/intball_common.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/intball_common/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/intball_common.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/intball_common/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/intball_common.svg?style=for-the-badge
[license-url]: LICENSE
