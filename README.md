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
