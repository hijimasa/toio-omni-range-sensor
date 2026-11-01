[English](README.md) | [日本語](README.ja.md)

# toio_omni_bringup

toio キューブ（`toio_ros2`）と全周距離センサ（`toio_omni_range`）を一緒に起動する。

**2 つのドライバは互いを知らない。TF でだけ出会う。**

```
map ──(toio_ros2: マットの Position ID)──> center ──(static)──> omni_link
                                                                    │
                                                          scan (LaserScan)
```

## 起動

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF
```

`device_address` はセンサ、`cube_id` はキューブ。**どちらも指定すること**を勧める
（理由は下の「BLE の取り合い」）。

| 引数 | 既定 | 内容 |
|---|---|---|
| `device_address` | `''` | センサの BLE アドレス。指定すると発見スキャンを飛ばす |
| `ble_adapter` | `''` | センサ側の BlueZ アダプタ（`hci1` など） |
| `cube_id` / `params_file` | `''` / A4 | `toio_ros2` にそのまま渡す |
| `sensor_x/y/z/yaw` | `0,0,0.0256,0` | `center` から見た鏡の位置 |
| `remap_cube_tf` | `false` | キューブの TF を `/tf_gt` に逃がす（測位ノードを足すとき）|
| `use_robot_state_publisher` | `false` | `toio_description` がある場合のみ |

## `goal_pose` は `cube_goal_pose` に remap してある

`toio_ros2` は `goal_pose` を購読して、キューブ内蔵の `motor_control_target`
で直接そこへ走る。**この指令は BLE でモータに直行し、`cmd_vel` を通らない。**
つまり collision_monitor もコストマップも一切効かない。

標準の `/goal_pose` に置いたままだと nav2 と競合し、RViz の 2D Goal Pose で
**キューブが自力で直進し、nav2 は空回りする**。実際にこれで回避が効かなかった。

なので内蔵の目標移動には専用の名前を与え、`/goal_pose` は実際に航行している
ものに任せる。キューブ内蔵の移動を直接使いたいときは:

```bash
ros2 topic pub --once /cube_goal_pose geometry_msgs/PoseStamped ...
```

## センサの取り付け

`sensor_yaw = 0`（センサの 0 度 = キューブ正面）は**検証済み**。

**`sensor_z` は実測値ではない**（キューブ上面 0.0256 m を置いてあるだけ）。
描画と 3D コストマップにしか効かないが、測ったら直すこと。

## BLE の取り合い

キューブ（toio-py → bleak）とセンサ（bleak）が同じ HCI アダプタを使う。
発見スキャンが同居する接続を不安定にするので、両方ともアドレス／ID を
指定してスキャンを避けるのが第一の対策。

**推測せず測る。** 手順:

```bash
# 1. センサ単体の基準値
ros2 launch toio_omni_range omni_range.launch.py device_address:=AA:BB:...
ros2 topic hz /scan --window 200
ros2 topic echo /diagnostics          # dropped / reconnects / rate_hz

# 2. 同じ測定を bringup で
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    device_address:=AA:BB:... cube_id:=a1b
```

見る数字:

| 出所 | 値 | 意味 |
|---|---|---|
| `/diagnostics` | `rate_hz` | 5 Hz（`scan_period` 既定 200ms）から落ちていないか |
| `/diagnostics` | `dropped` | 通知の欠落。seq のギャップから数えている |
| `/diagnostics` | `reconnects` | センサ側の再接続回数 |
| `toio_ros2` のログ | `toio is disconnected. reconnecting...` | キューブ側の切断 |

単体と同居で差が出なければ、それで終わり。差が出た場合の順番:

1. アドレス指定で発見スキャンを消す（まだなら）
2. `scan_period` を上げてセンサ側の帯域を落とす（`ble_client.py --cmd scan-period`）
3. USB ドングルを足して `ble_adapter:=hci1` で物理的に分ける

**1 プロセスに統合するのは最後の手段**で、そのためにドライバを 1 つに
まとめる（＝ `toio_ros2` をフォークする）必要はない。

## 回避層（`use_collision_monitor`、既定 false）

`nav2_collision_monitor` を `cmd_vel` の手前に挟む。自作しないのは、
スキャンの座標変換・古いデータのタイムアウト・急停止ではない減速が
すでに正しく実装されているため。

```bash
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    use_collision_monitor:=true device_address:=AA:BB:...

# 入力が cmd_vel_raw に移る。speed/turn を落とさないと（既定 0.5 m/s）鏡ごと倒れる
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw \
    -p speed:=0.03 -p turn:=1.8
ros2 topic echo /collision_monitor_state
```

既定を false にしているのは、有効にすると **teleop の入力先が `cmd_vel_raw` に
移る**ため。知らずに有効にすると「ロボットが反応しなくなった」ように見える。
nav2 launch では常に有効になる。

### 半径を決めているのはロボットではなくセンサ

| | 値 | 由来 |
|---|---|---|
| 死角 | 〜3.6 cm | `range_min`。ここより内側は存在しないのと同じ |
| footprint 半径 | 2.3 cm | 32 mm キューブの外接円。`robot_radius` と合わせる |
| `time_before_collision` | 2.0 s | 0.03 m/s で 6 cm 先読み。死角のすぐ外 |
| `simulation_time_step` | 0.02 s | 上限速度で 0.6 mm/step。5 mm の物体を跨がない |
| `min_points` | 2 | 小さい物体は数方向しか返さない（ステップ 3 の教訓）|

### 固定の停止円ではなく approach 型にしてある

書きやすいのは半径固定の stop ポリゴンだが、**20 cm のアリーナでは誤り**。
壁が常に数 cm 以内にあるので、停止円は端に寄っただけで発火し、しかも
stop は指令の向きを見ないので**後退して逃げることもできなくなる**。

`approach` は footprint を指令速度に沿って投影し、その動きが実際に
ぶつかる場合だけ介入する。壁から離れる方向はそのまま通る。

見える帯は 4.1〜11.1 cm の 7 cm しかない。5 Hz なので 5 cm/s で走ると
帯の横断に 1.4 秒 = 7 スキャン。**これがこの回避層で出せる速度の上限を
決めている。** 速く走らせるならセンサの `scan_period` を下げること。

数値はいずれも出発点。障害物に向けて走らせて `collision_monitor_state` を
見ながら詰めること。

## nav2

```bash
ros2 launch toio_omni_bringup toio_omni_nav2.launch.py \
    cube_id:=a1b device_address:=AA:BB:...
```

RViz の **2D Goal Pose** で目標を与える。`initialpose` は要らない
（マットの Position ID が絶対位置なので AMCL 自体が要らない）。

指令の経路:

```
controller_server / behavior_server ──> cmd_vel_raw
                                            │
                                     collision_monitor
                                            │  cmd_vel_gated
                                     cmd_vel_deadband
                                            │
                                         cmd_vel ──> toio_ros2 ──> キューブ
```

### モータのデッドバンド

`toio_ros2` の換算は `motor_speed = int(355.7 × 車輪速度)`。**キューブは
8 未満の値で車輪を回さない。**

| 指令 | 車輪速度 | motor_speed | |
|---|---|---|---|
| 1.5 rad/s のその場回転 | 0.0200 m/s | **7** | 回らない |
| 0.05 m/s の直進 | 0.0500 m/s | 17 | 回る |

**回転が丸ごと死ぬ。** DWB は経路に向けてまず旋回するので、そこで固まって
15 秒後に `Failed to make progress` になる。エラーはどこにも出ない。

`cmd_vel_deadband` が、速い方の車輪が 8 に届くまで**両輪を同じ率で**
スケールする（曲率は保たれる）。デッドバンドより上の指令は素通し。

**collision_monitor より後段**に置いているのは、`approach` が指令を
絞ったあとにまたデッドバンドを割り込むため。

実効的な最小その場回転は **1.69 rad/s**。`max_vel_theta` はその少し上の 2.0。
可動域の大半がデッドバンドだと「プランナが要求した旋回が実際には起きない」
状態が常態化するので下げられず、3.0 ではトップヘビーで倒れた。

### トップヘビーなので速度は安定性で決まる

32 mm のキューブの上にカメラ・ホルダー・鏡が載り、重心が高い。急発進・急停止・
その場旋回で倒れるので、**速度の上限はセンサではなく倒れないことで決めている。**
速度を出す経路は 3 つあり、どれか 1 つだけ下げても意味がない。

| 経路 | 設定 | 値 |
|---|---|---|
| nav2 の `cmd_vel` | `nav2_params.yaml` の `max_vel_x` / `acc_lim_*` | 0.03 m/s、0.15 m/s²、2.0 rad/s、8 rad/s² |
| キューブ内蔵の目標移動（`cube_goal_pose`、mapping が使う） | launch 引数 `goal_max_speed` | 15（motor 値。`toio_ros2` 既定の 30 ≒ 8 cm/s は倒れる） |
| teleop | `teleop_twist_keyboard` の `speed` / `turn` | 0.03 m/s、1.8 rad/s |
| nav2 のリカバリ（BackUp） | `behavior_trees/navigate_to_pose_toio.xml` の `backup_dist` / `backup_speed` | 0.03 m、0.025 m/s |

**4 つ目の経路は見落としやすい。** nav2 既定の Behavior Tree はリカバリで **0.30 m を 0.15 m/s** で
バックする。上限の 5 倍の速度でマット 1 枚ぶん下がる設定で、「いきなり速くバックした」の正体。
距離と速度は BT ノードのポートなので `nav2_params.yaml` からは変えられず、Jazzy の BackUp には
加速度や最低速度のパラメータも無い。そのため BT をパッケージ内にコピーして BackUp だけ
3 cm・0.025 m/s に変え、launch が `default_nav_to_pose_bt_xml` でそれを指す。
collision_monitor は後退も監視するが、後ろは `approach` の投影対象になるだけで速度自体は制限しない。

下げられないのはデッドバンドで、motor 値 8 未満は動かない。直進 0.0225 m/s、
その場回転 1.69 rad/s が「いちばん静かな発進」で、これより下は 0 か 8 かの二択。
collision_monitor の `time_before_collision` は `max_vel_x` との積が 6 cm になるよう
合わせてある（速度を変えたら一緒に変える）。

velocity_smoother は入れていない。`toio_ros2` が既に BLE 書き込みを
20 Hz に制限して重複を落としており、減速は collision_monitor 側が
ランプにしている。3 つ目の整形器を足す意味がない。

フレーム:

```
map ──(static identity)──> odom ──(toio_odom_bridge)──> center ──> omni_link
                                                          ▲
                                            toio_ros2 の TF は /tf_gt へ
```

`toio_ros2` の `map → center` は絶対位置で正しいが、nav2 は
`map → odom → base` の形を要求する。そこで `remap_cube_tf:=true` で
キューブの TF を `/tf_gt` に退避し、`toio_odom_bridge` が同じ `toio/pose`
から鎖を組み直す。**何も捏造していない。同じものを別の形で書いているだけ。**
`base_link` への改名はせず `center` をそのまま基準フレームにしている。

`toio_odom_bridge` が存在する理由は 2 つ:

- **キューブが止まると Position ID の通知も止まる。** 到着時ではなく
  タイマで TF を出し直さないと、nav2 が transform のタイムアウトで止まる。
- **DWB は odom の twist を現在速度として使う。** 1 mm 量子化の位置を
  微分すると大半がノイズなので、EMA をかけ、更新が止まったらゼロに落とす。

### マップは生成物

```bash
python3 tools/make_mat_map.py                       # A4
python3 tools/make_mat_map.py --params ../toio_ros2/params/toio_a3_play_mat_params.yaml
```

マット寸法は `toio_ros2` の params にしかない。手で描くと 3 つ目の写しが
できて、いずれ食い違う。**`params_file` を変えたらマップも作り直すこと。**

外周を occupied にしているのは意図的で、マットの外ではキューブが
Position ID を失って測位されなくなる。計画上そこは本当に壁。

### 32 mm のロボットのために変えた既定値

nav2 の既定はどれも建物の中の 50 cm のロボット用。単に保守的なのではなく
**この寸法では誤りになる**ものだけを変えてある。

| パラメータ | 既定 | ここ | 理由 |
|---|---|---|---|
| `required_movement_radius` | 0.5 | 0.008 | 0.297 m のマットで 0.5 m は進めない。全航行が stuck になる |
| costmap `resolution` | 0.05 | 0.005 | セルがロボットより粗い |
| `inflation_radius` | 0.55 | 0.035 | マット全体より大きい膨張 |
| `max_vel_x` | 0.26 | 0.03 | トップヘビーで倒れない上限（下記）。センサだけなら 0.05 でも追いつく |
| `acc_lim_x` / `acc_lim_theta` | 2.5 / 3.2 | 0.15 / 8.0 | 急発進・急停止で倒れる |
| `max_vel_theta` | 1.0 | 2.0 | デッドバンド 1.69 rad/s の少し上。3.0 では倒れた |
| `xy_goal_tolerance` | 0.25 | 0.008 | マットの縦の 1/8 が許容誤差では困る |
| `Oscillation.*` | 0.05 | 0.02 | 普通の動作が振動と判定される |

**`obstacle_max_range` を `range_max` より小さくしている**のが分かりにくい点。
`inf_is_valid: True` にすると nav2 は inf を `range_max` に置き換えるので、
`obstacle_max_range` を `range_max` と同じにすると**地平線上に幻の障害物が
並ぶ**。0.225（marking）< 0.233（range_max）< 0.240（raytrace）としてある。
これで「何も無い方向」がちゃんと消去に使われる。

### 動かないときの切り分け

**`Robot to stop due to invalid source` が出ていたら、それが答え。**
スキャンが来ていないので collision_monitor が止めている。nav2 側は正常。

```bash
ros2 topic hz /scan                  # 何も出なければセンサが繋がっていない
ros2 topic echo /collision_monitor_state
```

センサ抜きで nav2 だけ試すには:

```bash
ros2 launch toio_omni_bringup toio_omni_nav2.launch.py use_collision_monitor:=false
```

これで controller が直接 `cmd_vel` に出す。**動くなら問題はセンサ側**。
ただし当然ながら回避は無いので、確認が済んだら戻すこと。

その他:

| 症状 | 見るところ |
|---|---|
| `/plan` は出るが `/cmd_vel` が空 | collision_monitor（上記）|
| `/plan` も出ない | planner か global_costmap、TF |
| `Failed to make progress` | 15 秒動かなかった結果。原因は別にある |
| `parameter ... is of type {integer}` で即死 | コストマップの `width`/`height` は整数 |

## `remap_cube_tf` について

`toio_ros2` に `publish_tf` パラメータがあればこの remap は要らない。
外部ローカライザを使う人に一般に有用なので、upstream への小さな PR に向く。
