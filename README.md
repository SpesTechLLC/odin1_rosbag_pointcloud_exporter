# Odin1 ROS 2 Bag Point Cloud Exporter

Odin1がROS 2 bagへ記録した着色点群 `/odin1/cloud_slam` をオフラインで統合し、バイナリPCDまたはPLYとして保存するツールです。

- 点群を指定座標系へ変換して時系列で統合
- XYZとRGBを保持
- CloudCompareで開きやすいバイナリPLYに対応
- 5 cmのグローバルVoxelフィルタを既定で適用
- ディスク分割処理により、大きなbagでも全点をRAMへ保持しない
- Voxelフィルタなしの全点出力にも対応

## 必要な環境

- ROS 2（Humbleで動作確認済み）
- Python 3
- `rosbag2_py`
- `rclpy`
- `sensor_msgs`
- `numpy`
- MCAP形式のbagを読む場合はROS 2のMCAP storage plugin

通常はROS 2環境をsourceすれば必要なPythonモジュールを利用できます。

```bash
source /opt/ros/humble/setup.bash
```

## 入力bagの要件

既定では次のtopicと座標系を使用します。

- 点群topic: `/odin1/cloud_slam`
- PointCloud2フィールド: `x`, `y`, `z`, `rgb`または`rgba`
- 出力座標系: `odom`

Odin1の `/odin1/cloud_slam` は通常、すでに `odom` 座標で配信されます。この場合は追加のTFを適用せず、各時刻の点群をそのまま統合します。入力点群と出力の座標系が異なる場合は、bag内の `/tf` と `/tf_static` を読み、点群時刻に合わせた変換を適用します。

## 使い方

このディレクトリへ移動して実行します。

```bash
./export_odin1_pointcloud.py /path/to/rosbag_directory \
  -o odin1_colored_map.ply
```

出力形式は拡張子から自動判定されます。

- `.ply`: binary little endian、`x y z red green blue`
- `.pcd`: binary、`x y z rgb`（RGBはpacked uint32）

CloudCompareで使用する場合は、PCD pluginの有無に影響されないPLYを推奨します。
`-o`を省略した場合は、現在のディレクトリへ `<bag名>_odin1_colored_map.ply` を出力します。

## Voxelフィルタ

既定では、5 cmのグローバルVoxelフィルタを適用します。

```bash
./export_odin1_pointcloud.py /path/to/rosbag_directory \
  -o odin1_colored_map.ply \
  --voxel-size 0.05
```

全点を一切間引かずに保存する場合は `--voxel-size 0` を指定します。出力ファイルが非常に大きくなる可能性があります。

```bash
./export_odin1_pointcloud.py /path/to/rosbag_directory \
  -o odin1_colored_map_all_points.ply \
  --voxel-size 0
```

## 主なオプション

```text
--topic TOPIC            点群topic（既定: /odin1/cloud_slam）
--target-frame FRAME     出力座標系（既定: odom）
--voxel-size METRES      Voxelサイズ。0で無効（既定: 0.05）
--partitions N           Voxel処理用の一時ディスク分割数（既定: 256）
--temp-dir DIR           一時ファイルの保存先
--every-nth-frame N      Nフレームごとに処理
--start-offset SECONDS   最初の点群から指定秒後に処理開始（既定: 0）
--end-offset SECONDS     最初の点群から指定秒後に処理終了
--max-tf-gap SECONDS     TF補外を許容する最大時間（既定: 0.5）
--force                  既存の出力ファイルを上書き
```

すべてのオプションは次のコマンドで確認できます。

```bash
./export_odin1_pointcloud.py --help
```

## 大きなbagを処理するとき

Voxel処理では、点を空間ハッシュで複数の一時ファイルへ分割し、一分割ずつ重複除去します。RAM使用量はbag全体の点数に比例して増えませんが、一時領域には数GiB以上必要になることがあります。空き容量が十分な場所を `--temp-dir` で指定してください。

## カメラパスYAMLの生成

`make_camera_path_from_bag.sh` はOdinのodometryから、点群ビューア向けのカメラパス（`schemaVersion: 2`）を生成します。元bagは変更しません。

追加の依存関係:

- `nav_msgs`（ROS 2）
- Pythonの`scipy`、`PyYAML`（`numpy`も使用）

shは既定で`/opt/ros/humble/setup.bash`を読み込みます。別のROS環境を使う場合は`ROS_SETUP=/path/to/setup.bash`を指定してください。

### 記録開始から終了時刻まで

```bash
./make_camera_path_from_bag.sh /path/to/rosbag_directory \
  --start 0 --end 14:11:30 \
  --interval 10 --duration 3801 \
  --output /path/to/Camera-path.yaml
```

この例では、記録開始から同日の14:11:30までを約10秒間隔で抽出し、再生時間を3801秒にします。実際に処理した`20260923130808`では、記録開始13:08:09.014282 JSTから14:11:30まで約3800.986秒、始終点を含め382点でした。

`--start` / `--end`は次の形式を指定できます。

| 形式 | 例 | 意味 |
|---|---|---|
| 経過秒 | `600` | bag記録開始から600秒後 |
| 時刻 | `14:11:30` | bag開始日の指定タイムゾーンでの時刻 |
| ISO日時 | `2026-09-23T14:11:30+09:00` | 日付と時差を含む日時 |

日を跨ぐ場合はISO日時を使用してください。`--timezone`の既定は`Asia/Tokyo`です。時刻指定には**bagの記録timestamp**を使い、Odinデバイスのheader timestampとは区別します。点群exporterの`--start-offset` / `--end-offset`は最初の点群を基準とするため、基準が異なります。

```bash
# 13:20～14:00の区間。再生時間は区間長から自動設定。
./make_camera_path_from_bag.sh /path/to/bag \
  --start 13:20:00 --end 14:00:00 --output segment.yaml

# 記録開始10分後～20分後を60秒で再生。
./make_camera_path_from_bag.sh /path/to/bag \
  --start 600 --end 1200 --interval 2.5 --duration 60 --output short.yaml
```

### 主なオプション

| オプション | 既定値・動作 |
|---|---|
| `--start` | `0`（bag記録開始） |
| `--end` | bag記録終了 |
| `--interval` | `10`秒。元計測時間での最大点間隔 |
| `--duration` | 省略時は選択区間の実時間。指定すると再生時間を変更 |
| `--topic` | `/odin1/odometry`（`nav_msgs/msg/Odometry`） |
| `--timezone` | `Asia/Tokyo` |
| `--output` | 必須。出力YAMLのパス |
| `--cached-csv` | 抽出済み`.source.csv`を利用してbagの再読み込みを省略 |
| `--force` | 既存のYAMLと付随ファイルの上書きを許可 |

始点・終点を含む均等時間間隔になるよう分割するため、実際の点間隔は指定値以下になります。位置は線形補間し、境界姿勢はSLERPで補間します。選択した途中区間の境界にデータがない場合は失敗します。bagの開始・終了だけは最大0.5秒まで端のposeを保持し、その時間を`.range.json`に記録します。

### 保存されるファイル

`Camera-path.yaml`に対して、次も同じディレクトリに保存します。

- `.source.csv`: 抽出元軌跡。`timestamp`はbag記録時刻、`header_timestamp`はデバイス時刻
- `.trajectory.csv`: 選択区間の境界を補間した軌跡
- `.timing.csv`: 各waypointの区間開始からの経過秒・記録timestamp・位置
- `.range.json`: 区間・座標系・補間条件・件数
- `.generation.json`: カメラパス生成結果

大容量bagを読み直さずに、取得済み範囲内を再指定できます。bagのmetadataは引き続き必要です。

```bash
./make_camera_path_from_bag.sh /path/to/bag \
  --cached-csv Camera-path.source.csv \
  --start 600 --end 1200 --interval 10 --output cached-path.yaml
```

キャッシュに対応する`.range.json`がある場合、bagとtopicが一致することも検証します。別の場所へ移動したbagやtopicの異なるキャッシュは再抽出してください。

### CSVから直接生成・角度の調整

`make_camera_path.sh`はROS環境なしでも実行できます。入力は`timestamp,x,y,z,qx,qy,qz,qw`形式のCSV（少なくとも時刻とXYZが必要）です。

```bash
./make_camera_path.sh --input trajectory.csv --output Camera-path.yaml \
  --interval 10 --duration 3801 --max-pitch 10 --height-offset 0
```

カメラは前後2.5秒の移動方向を1秒幅で平滑化して向けます。停止中は前後の移動方向を補間し、全区間に水平移動がない場合はエラーになります。ロールを抑え、ピッチは既定±10度です。これはセンサー姿勢の再現ではなく、見やすい進行方向追従です。`--heading-window`、`--smoothing`、`--stationary-speed`、`--max-pitch`、`--height-offset`、`--look-distance`、`--fov`で調整できます。

- 元の座標を保持し、PGO地図への位置合わせは追加しません。
- quaternionは`[x, y, z, w]`、カメラ前方は−Z、world上方向は+Z。
- targetは既定10 m先、fovは60。IDはUUID4で生成します。
- YAMLにはwaypointごとの時刻欄を追加していません。ビューアの補間方式によって区間速度は実測と一致しない場合があります。

### 検証

小さなMCAPを生成し、記録時計とデバイス時計の区別、3形式の時間指定、境界補間、カメラ軸、キャッシュ、上書き拒否を検証します。

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
```
