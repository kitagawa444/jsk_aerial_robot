# 力推定・力制御の試験

このページは現在の汎用BeeController用です。運搬と力推定の精度は別に検証します。
試験値は実機の安全範囲や接触力精度の保証ではありません。

## 実験ごとの起動と操作

実機では各端末が同じ実機用 ROS master を参照する。手動 MuJoCo は専用 master を使い、Joy・計測端末も同じシミュレーション master に接続する。例えばローカル MuJoCo 用に全端末で `export ROS_MASTER_URI=http://localhost:12930` を設定し、1端末で `roscore -p 12930` を起動する。各行の常駐プロセスは別端末で実行する。

### 1. 単機で observer と力計を比較する

実機（Bee 1、mocap 推定）と操縦 PC の起動：

```bash
roslaunch bee bringup.launch robot_id:=1 estimate_mode:=1
roslaunch bee joy_stick.launch robot_count:=1
```

力計のドライバは使用機器に合わせて別途起動し、荷重の単位・軸・トピック名を確認する。先に bag 記録を開始する（下記）。単機 MuJoCo の手動操作では1行目を以下に置き換える。

```bash
roslaunch bee bringup.launch robot_id:=1 real_machine:=false simulation:=true mujoco:=true force_test_wall:=true headless:=false
```

静止ホバーで姿勢目標トピックに pitch +90° を指定する（world yaw = 0 を使う配置の例。実際は力計への方位に合わせる）。

```bash
rostopic pub -1 /bee1/final_target_baselink_rpy geometry_msgs/Vector3Stamped   '{header: {stamp: now, frame_id: world}, vector: {x: 0, y: 1.5707963, z: 0}}'
```

#### 接触前の校正

試験1・2とも、pitch +90° の姿勢が安定し、足が力計に触れていない状態で行う。

1. 端末Aで校正状態の表示を開始する。

   ```bash
   rostopic echo /bee1/controller/observer_calibration
   ```

2. 端末Bから校正を1回要求し、完了まで静止ホバーを保つ。

   ```bash
   rostopic pub -1 /bee1/controller/calibrate_observer std_msgs/Empty '{}'
   ```

3. 端末Aで、要求後に `state: 1`（収集中）から `state: 2`（成功）に変わり、`calibration_stamp` が更新されたことを確認する。`detail` に `hover Z trim saved` が出れば、鉛直推力のホバー補正分も保存された。

試験1では、校正後に Joy で力計に近づき、ゆっくり押して戻す。手動操作中は `/bee1/command` を送らない。

`force_test_wall:=true` で単機 MuJoCo に固定の計測壁を追加する。機体側の面は world の `x = −0.75 m`、幅は Y 方向に 1.20 m、高さは床から 1.50 m。初期位置から −X へ進むと、pitch +90°、yaw 0° で足先が壁を向く。壁を使わない通常の起動では引数を省略できる。

壁が受けた接触力の合計は `/bee1/mujoco/wall_force`（`geometry_msgs/Vector3Stamped`）に world 座標・N で出る。無接触でゼロ、押すと `vector.x` が負、壁を上にこすると `vector.z` が正になる。床との接触は含まない。

```bash
rostopic echo /bee1/mujoco/wall_force/vector
```

### 2. 単機で力追従と力計を比較する

**目標値の記入例**。world の −X 側に鉛直の力計を置き、pitch +90°、yaw 0° で接触する配置を想定する。A は水平に 0.50 N、B は水平に 0.50 N と上向きに 0.10 N。これはコマンドの例で、実機で検証した荷重ではない。壁が +X 側にある場合は接触姿勢を合わせ、Fx の符号を反転する。

| 用途 | トピック | 型・値 |
|---|---|---|
| 姿勢を合わせる | `/bee1/final_target_baselink_rpy` | `geometry_msgs/Vector3Stamped`、world、試験例 (0, 1.5707963, 0) rad |
| 無荷重で校正する／結果を見る | `/bee1/controller/calibrate_observer` → `/bee1/controller/observer_calibration` | `std_msgs/Empty` → `bee/ObserverCalibrationStatus`。今回の state=2 を確認 |
| **目標力を送る** | **`/bee1/command`** | **`bee/BeeCommand.force`、world、Bee→板、単位 N。A = (−0.50, 0, 0)、B = (−0.50, 0, +0.10)** |
| 校正後の推定力を見る | `/bee1/controller/tracking_status` | `bee/ControllerStatus.external_force`、world、板→Bee。目標との比較では符号反転 |
| 校正前の生出力を記録 | `/bee1/estimated_external_wrench` | `geometry_msgs/WrenchStamped`、CoG座標 |
| 力誤差・飽和を見る | `/bee1/controller/force_feedback` | `bee/ForceFeedback` の force_error、correction_saturated、acceleration_limited |
| 高度を読む | `/bee1/uav/cog/odom` | `nav_msgs/Odometry.pose.pose.position.z`、world、m |
| 独立した接触力を記録 | **使用する力計ドライバの出力** | Fx と Fz を独立に測る。実トピック名・軸・符号・型は力計側で確認 |
| 追従を緊急停止 | `/bee_cooperative_grasp/emergency_stop` | `std_msgs/Empty`。同じmasterの購読機が対象 |

**実行順**。試験1と同じ単機 bringup と Joy launch を起動する。Joy launch が L2＋R2 の停止ボタンも受け付ける。Joy で押している間は command を送らない。

```bash
rostopic info /bee_cooperative_grasp/emergency_stop
rostopic info /bee1/command
```

停止トピックにJoyブリッジとBee 1、command にBee 1の購読があり、ほかに指令送信元がないことを確認する。記録を開始し、pitch +90° で[接触前の校正](#接触前の校正)を行ってから、Joy で板に接触する。以下は Bash で使う送信例。A/B とも `-s --use-rostime` により時刻を毎回更新する。壁に触れていない状態で送ると、押し付け分が機体の加速度になる。

```bash
send_force() {
  rostopic pub -r 50 -s --use-rostime /bee1/command bee/BeeCommand "{
    header: {stamp: now, frame_id: world},
    navigation: {control_frame: 0, target: 1, pos_xy_nav_mode: 3, pos_z_nav_mode: $1, target_pos_z: $2},
    baselink_rpy: {x: 0, y: 1.5707963, z: 0},
    enable_force_control: true, force_integral_enabled: true,
    force: {x: $3, y: 0, z: $4}, velocity_point: {x: 0.0175, y: 0, z: -0.143}}"
}
```

**試験A**。接触直前のホバー高度を1回読む。値を確認し、接触後に同じ端末で送信する。XY=ACC、Z=POS。

```bash
z_hold=$(rostopic echo -n 1 /bee1/uav/cog/odom/pose/pose/position/z | sed -n '1p')
echo "$z_hold"
send_force 2 "$z_hold" -0.50 0.0
```

**試験B**。独立した試行として姿勢・校正・接触を確認し、水平の接触が成立した状態から送信する。XY=ACC、Z=ACC。第2引数の0はこのモードでは使用しない。高度保持は入らない。Yは両試験とも速度減衰のみで、固定位置の指令は入れていない。

```bash
send_force 3 0 -0.50 0.10
```

**振動時は L2＋R2**。端末なら `rostopic pub -1 /bee_cooperative_grasp/emergency_stop std_msgs/Empty '{}'`。Bee は追従をラッチ停止し、今回の指令ではその場の位置保持に戻る。停止後に送信端末の Ctrl-C で連続 publish も終了する。単に publish を途切れさせても参照タイムアウトになる。次の独立試行は送信元の停止を確認し、参照受付の再初期化後に行う（`/bee1/controller/set_reference_mode` を false → true）。

#### FF だけで水平に押す試験

試験1と同じ単機 bringup・Joy・姿勢・校正を使う。接触後、次の例は world −X に 0.50 N 相当の加速度を指令する。Z は開始時の高度を保持する。XY は ACC なので Y の位置保持は入らない。力の換算には起動中の Bee が URDF から求めた質量を使う。力計で測る値と指令値はずれ得る。

```bash
send_ff_x() {
  local mass ax z_hold
  mass=$(rosparam get /bee1/controller/reference_tracking/model_mass) || return 1
  ax=$(python3 -c 'import sys; print(float(sys.argv[1]) / float(sys.argv[2]))' "$1" "$mass") || return 1
  z_hold=$(rostopic echo -n 1 /bee1/uav/cog/odom/pose/pose/position/z | sed -n '1p')
  [ -n "$z_hold" ] || return 1
  rostopic pub -r 50 -s --use-rostime /bee1/command bee/BeeCommand "{
    header: {stamp: now, frame_id: world},
    navigation: {control_frame: 0, target: 1, pos_xy_nav_mode: 3, pos_z_nav_mode: 2,
                 target_acc_x: $ax, target_pos_z: $z_hold},
    baselink_rpy: {x: 0, y: 1.5707963, z: 0}, enable_force_control: false}"
}
send_ff_x -0.50
```

FF 指令力は `model_mass × command.navigation.target_acc_x` で復元する。この試験で `command.force` はゼロのまま。observer は記録用で、力誤差のフィードバックには使わない。**解除は L2＋R2**。その場の位置保持に戻ったら、送信端末で Ctrl-C を押す。解除がラッチされるため、ボタンを離しても押し付けは再開しない。次の試行前は送信を止めた状態で以下を実行する。

```bash
rosservice call /bee1/controller/set_reference_mode 'data: false'
rosservice call /bee1/controller/set_reference_mode 'data: true'
```

MuJoCo では、試験1の壁付き起動を使い、FF 指令力・`/bee1/mujoco/wall_force/vector/x`・`−/bee1/controller/tracking_status/external_force/x` を同じ符号で比較する。

### 3. 三角柱を3機で運搬する

実機は各機のホストで対応する番号の bringup を起動する。以下の中央と Joy は各1回。物体 mocap の既定 ID は10。

```bash
roslaunch bee bringup.launch robot_id:=1 estimate_mode:=1
roslaunch bee bringup.launch robot_id:=2 estimate_mode:=1
roslaunch bee bringup.launch robot_id:=3 estimate_mode:=1
roslaunch bee cooperative_grasp.launch execute:=true object_model:="$(rospack find bee)/config/grasp/object_triangle_real.yaml"
roslaunch bee joy_stick.launch robot_count:=3
```

MuJoCo では下記の2本で、3機・実CAD三角柱・中央・Joyがそろう。手動開始のため auto_start は false。

```bash
roslaunch bee grasp_demo.launch robot_count:=3 world:=0066 headless:=false auto_start:=false
roslaunch bee joy_stick.launch robot_count:=3
```

全機 HOVER 後に L1＋R1 で開始。Approach／Preload 中は R1＋□ で再整列し、整列後に L1＋R1 で再開。L2＋R2 は協調中止。中央が姿勢合わせと校正を行う。運搬中の状態と判断理由は以下で確認する。

```bash
rostopic echo /bee_cooperative_grasp/phase
rostopic echo /bee_cooperative_grasp/reason
rosbag record -O transport /mocap/object/pose /bee_cooperative_grasp/{phase,reason,preload_diagnostics,emergency_stop} /bee{1,2,3}/{command,estimated_external_wrench,controller/force_feedback,controller/tracking_status,uav/cog/odom} /tf /tf_static
```

## 単機で行う最小の実験

**1機＋力計（ロードセル）で、推定と制御を別々に測ります。**
以下は実機試験の計画であり、飛行・モーター回転は今回実施していません。
押し付け方向をworldの単位ベクトル`d`、力計が測る押し付け力を`F_meas`とします。

| 試験 | 与える指令・荷重 | 比較 |
|---|---|---|
| 推定精度 | 通常の位置・姿勢保持、`enable_force_control=false`。力計付きの糸などで既知の外力を段階的に加える | observerの推定値と力計の値。一定荷重中の平均誤差・標準偏差、変化への遅れ |
| 押し付け精度 | 力計付き固定面へ接触してから`enable_force_control=true`。`force=F_target*d`、例として0.5→1→1.5→0.5 N、各10秒、3往復 | **力計の値と目標力**。定常誤差・立上り時間・オーバーシュート・振動。observerも重ねる |

荷重の値は試験案です。まず同じ姿勢・無荷重で一度校正し、測定中は取り直しません。
まず押し付け方向の1軸で実施し、3次元の推定精度が必要ならXYZ各方向で繰り返します。
力計なしでは「observerに追従したか」までしか評価できません。生observerはCoG座標・環境→Bee、
押し付け指令はworld座標・Bee→環境なので、座標変換と符号を合わせます。

必要な上位指令は`/bee1/command`（`bee/BeeCommand`）へ50 Hz、
各回の`header.stamp`を更新し`header.frame_id=world`で送ります。

- Joy で手動押し付けする推定試験では、上位から BeeCommand を送らない。
- 力フィードバックを使う試験 A/B は `enable_force_control=true`、PIを使う場合は `force_integral_enabled=true`。上記の壁では押し付け方向が world −X、上が +Z。navigation は WORLD_FRAME、COG、回転3軸は NO_NAVIGATION。baselink_rpy は校正時の接触姿勢、velocity はゼロ、velocity_point はモデルの足の代表点を使う。力は Bee が面に加える N 単位の値。運動用加速度に force/m や g を加算しない。

| 試験 | pos_xy_nav_mode | pos_z_nav_mode | force (world) | Z の目標 |
|---|---|---|---|---|
| A：水平押し付け＋高度保持 | ACC_MODE | POS_MODE | (Fx, 0, 0) | 力制御開始時の CoG の world Z を1回保存し、全周期で同じ target_pos_z を送る |
| B：水平押し付け＋鉛直荷重 | ACC_MODE | ACC_MODE | (Fx, 0, Fz) | 運動用 target_acc_z=0。鉛直力制御と速度減衰を使用し、絶対高度の目標は入れない |

両試験とも target_acc_x=0。Y 位置を維持する場合は上位が Y の位置・速度誤差から target_acc_y を与える。A でも力PIは3軸で、Fz目標はゼロ。下流はZの力補正を高度PIDへの加速度フィードフォワードとして加える。

計測には壁の接触板を通る法線力と鉛直せん断力を独立に読める2軸以上の力計を使う。接触板を取り付けた状態で無荷重ゼロを取り、センサ軸を world に変換して Bee→面の符号にそろえる。B は Fx を安定させてから Fx を一定に保って Fz をゼロから増やし、最後に Fz を先にゼロに戻す。滑りを避けるには計測される法線力に対して接線力が静止摩擦の範囲内にある必要がある。

各軸で /bee1/command.force、-/bee1/controller/tracking_status.external_force（校正・フィルタ済みのworld外力の符号を反転）、力計の同時波形を比較する。A は CoG の高度誤差、B は高さの変化と接触点の滑りも記録する。振動時は /bee1/controller/force_feedback の correction_saturated と acceleration_limited を合わせて確認する。
- 接触前は位置制御で直線接近し、接触を確認してから力指令へ切替。法線位置保持は重ねない。

実機・シミュレーションとも、Bee の起動時から外力 observer と自重補償が有効です。力追従は各 `/bee1/command` の `enable_force_control` で指定します。
単機試験の FB では `send_force` が力を、FF だけの試験では `send_ff_x` が換算した加速度を送ります。単機自動runnerは推定用simのみです。
既存の`control`試験は下記のとおり4機・箱用です。

接触前の静止ホバーで校正すると、同じ観測区間の鉛直推力指令（加速度換算）からモデルの重力を引いた平均も保存します。成功時の `detail` に `hover Z trim saved` と値が出ます。鉛直 ACC ではその値を重力に加えます。位置制御での高さ保持以外で校正した場合は observer のみ更新し、直前の補正値を保持します。制御器のリセットで補正値はゼロに戻ります。

記録は校正前に開始する。以下は壁付き MuJoCo 用。実機では最後の `/bee1/mujoco/wall_force` を実際の力計トピックに置き換える。

```bash
rosbag record -O single_bee_force.bag \
  /bee1/estimated_external_wrench /bee1/command \
  /bee1/controller/tracking_status /bee1/controller/force_feedback \
  /bee1/controller/force_control_parameters /bee1/controller/observer_calibration \
  /bee1/uav/cog/odom /bee1/uav/baselink/odom /tf /tf_static /bee1/mujoco/wall_force
```

既存のbag解析はobserverに対する追従の解析です。力計との精度比較は、使用する力計の
メッセージ型・時刻・符号に合わせた対応が別途必要です。

## 一括実行（MuJoCo・隔離ROSのみ）

ROS環境をsourceして、リポジトリ直下から実行します。

```bash
python3 robots/bee/test/run_force_tests.py --output /tmp/bee-force-tests --port 12971
```

`--checks contract observer control`で対象を選べます。出力先は新しいディレクトリを指定します。
既存masterへ接続せずlocalhostの専用masterを起動し、終了時に停止します。11311は拒否します。
実機操作には使えません。`contract`にはビルド済み`reference_tracker_harness`が必要です。

| 試験 | 内容 | 判定・成果物 |
|---|---|---|
| contract | 合成odometry/observerで本物のReferenceTrackerを動かす | 運動・力追従・校正・設定変更・途絶停止など28項目。モーター出力なし |
| observer | 接触なしの単機ホバリングで、MuJoCo rootへ既知のworld外力±0.4 NをXYZに加える | 固定した校正値で推定外力と比較。各区間の先頭2秒を除いた3D RMSE≤0.2 N、接触なし、入力鮮度を確認 |
| control | 4機・箱のHold中に釣り合う接触力指令を3方向へ励起 | 指令とobserverの追従誤差、波形、応答モデル。既定3D RMSE≤0.5 N |

RMSE上限は試験用の明示的な判定値です。`--observer-rmse-limit` / `--control-rmse-limit`で指定できます。
controlは**observerに対する追従**を評価します。observerと真の接触力のずれまで解決したことにはなりません。
MuJoCoの足site forceには足の重量・慣性も含まれるため、その値を真の接触力として代用しません。

observerの単独実行:

```bash
python3 robots/bee/test/run_observer_force_test.py --output /tmp/bee-observer --port 12931
python3 robots/bee/script/analyze_force_validation.py /tmp/bee-observer/flight.bag \
  --mode observer --output /tmp/bee-observer-analysis
```

`run_observer_force_test.py`はprivate master、use_sim_time、MuJoCoプラグイン、接続状態を確認してから
シミュレータ内の離陸と外力入力を行います。通常の位置制御でホバリングし、力PIを使って推定値を目標へ
引き寄せることはしません。励起中の再校正も禁止した試験プロトコルです。

## 保存bag・実機への再利用

`script/record_grasp_force_response.py`は記録専用です。機体の起動、arm、目標力変更は行いません。
実機では既存の回転禁止など、その時点の運用条件を守ってデータを取得してください。

```bash
python3 robots/bee/script/record_grasp_force_response.py --source hardware \
  --names bee1 --label airframe-and-firmware-id --duration 120 --output /tmp/bee-force-record
python3 robots/bee/script/analyze_force_validation.py /tmp/bee-force-record/flight.bag \
  --mode control --names bee1 --output /tmp/bee-force-analysis
```

解析はROS masterへ接続しません。BeeCommand/ControllerStatusを優先し、指定した機体の旧Grasp参照/statusも読めます。
力の向きは指令がBee→環境、observerが環境→Beeなので符号を反転して比較します。
時刻を過去サンプルで合わせ、欠測・古い参照・校正変更で誤差を隠さず、CSV/PNG/JSONを出します。
observerモードは既知外力と区間マーカーがあるシミュレーション用です。通常の実機bagだけから
真の接触力を推定して「推定精度」を判定する機能ではありません。

応答時間の詳しい同定は既存の[FORCE_IDENTIFICATION.md](FORCE_IDENTIFICATION.md)を参照してください。
`run_force_tests.py`のcontrolはその同定スクリプトも実行します。
4機×3方向の12モデルについて採用条件も検査します。追従誤差が小さくても、他方向の入力が
同時に大きく変わる場合などは、その方向だけの応答時間を同定できたとは扱わず、試験全体を非0で終了します。
`summary.json`で運転完了・追従誤差・同定品質の判定を個別に確認してください。

## 複数形状の運搬回帰

```bash
python3 robots/bee/test/run_transport_repeats.py --output /tmp/bee-shapes \
  --repeats 1 --workers 1 --port 13001
```

対象は三角柱、箱、五角柱、球、楕円体、円柱、カプセル、岩形状、球の下方支持の9条件。
既存のユーザー指定に従い円錐系は通常の一覧から外しています。
`--shapes triangle box`のように絞れます。各試験に異なるportを使い、前のmasterのTIME_WAITを避けます。
QPにはwall time 10 msの計算予算があります。基準試験は`--workers 1`で行い、
並列実行は計算負荷を含む試験として区別してください。
1条件でも未完走・起動失敗・解析失敗なら終了コードを非0にし、結果を`results.json`へ残します。
終了コード0は試験全件のCompleteを意味します。各1回の結果は反復安定性の保証ではありません。

`protocol.json`には実行に使う制御ソース・設定・主要バイナリのhash、`schedule.json`には実行順序を記録します。
入力変更は`completion.json`で検出します。既知の制御失敗を、試験起動の失敗やログ欠損と混同しないでください。
