# 実機3台のobserver把持デモ

`real_grasp_demo.py`は、従来の3台キーボードデモの古典制御を再利用する実機入口です。
接近→4球で押し付け→キーで持ち上げ→HOLD後のXYZ/yaw操作を対象とします。
自動運搬・台座への設置シーケンス、4/5台、MPCはこの入口の対象外です。
実機飛行・実機ゲイン調整は未検証です。MuJoCoでの成功を実機安全性の保証には使わないでください。

## 入力・座標系

| 入力 | 既定topic | 要件 |
| --- | --- | --- |
| 機体位置・姿勢 | `/beeN/uav/baselink/odom` | baselinkの姿勢・位置、線速度はworld座標（このrepoのestimator出力規約） |
| 重心位置・姿勢 | `/beeN/uav/cog/odom` | world座標、child frameは対応する`beeN/cog` |
| 推定外レンチ | `/beeN/estimated_external_wrench` | 対応するCoG座標系で表現された機体が受ける外力・モーメント |
| 飛行状態 | `/beeN/flight_state` | 実行時は全台HOVER=5、周期publish |
| 物体姿勢 | `/mocap/grasp_object/pose` | PoseStamped、world座標、原点は物体の幾何中心 |

物体のmocapマーカー原点が中心からずれている場合、上流で剛体変換してください。
この入口はTF変換・原点補正を自動推定せず、不一致frameを拒否します。
三角柱の側面法線は物体XY座標で60/180/300度、柱の長手方向は物体Zです。
機体と物体のmocap座標系、原点・軸方向、時計同期を必ず揃えてください。
`robot_namespaces`で論理bee1/2/3を実機namespaceへ対応させ、`cog_frames`も実値に合わせます。
`odometry_topics`を使う場合もbaselink/world線速度規約は変わりません。

force-site/touch/ばね変位の購読、ground_truth、外力注入、MuJoCo backendは使いません。
未取得のforce-site診断CSVは0ではなくNaNです。診断topicの一部には互換性のため
`/mujoco_triple_press/`という従来名が残りますが、シミュレータへの操作ではありません。

## 準備

1. `real_grasp_demo.yaml`をコピーし、物体質量・辺長・高さ、機体質量、足のオフセット・球半径・圧縮量、topic/frameを実測値に置換してください。既定数値は例であり実機同定値ではありません。
2. 各Beeを通常の実機起動手順で起動し、`bee bringup.launch`に追加した
   `grasp_gravity_compensation:=true`を指定してください（`robot_id:=1/2/3`）。
   通常の実機PID設定は変更していません。このフラグはcontroller初期化前に重力補償を有効化します。
   実行中の`rosparam set`だけではcontroller内部へ反映されないため、必ず起動時に設定します。
3. このブランチのZ ACC対応navigation/controllerを使用してください。実機でZ ACCを使うと
   hover積分項がリセットされるため、重力補償OFFのまま実行しないでください。
   DAブランチの制御バイナリがdevelに残っている可能性もあるため、ブランチ切替後は再ビルドが必要です。
4. 最初は非飛行でtopic/frameを確認します（observerが地上で配信されない場合、全入力preflightは未成立になります）。次に単機で重力補償・ACC方向・roll90度姿勢保持・observerの符号を確認します。
   各機の質量差を扱う制御は未対応（共通robot_mass）。同等機体・足構造を前提にしてください。
5. 全台をそれぞれの側面側に離して配置し、移動経路と台座・プロペラのクリアランスを確認します。
   接近経路に対する障害物回避はありません。機体同士が交差する初期配置で開始しないでください。

## コマンド

環境をsourceした上で、まず入力確認だけを実行します。飛行指令は送りません。

```bash
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch bee real_grasp_check.launch config:=/absolute/path/real_grasp_demo.yaml
```

各機の通常の操縦手順で離陸・HOVERさせます。デモからarm/takeoffは送りません。
操縦者による独立したRC override・緊急対応、物体落下対策を用意し、設定と経路を確認した後、
対話端末で実行します（ロボットを動かすコマンドです）。

```bash
rosparam load /absolute/path/real_grasp_demo.yaml /bee_real_grasp_demo
rosrun bee real_grasp_demo.py _execute:=true _configuration_verified:=true
```

開始すると待機位置への移動・roll90度への移行・接触前observerオフセット計測・低速接近を行い、
押し付け状態になります。`w/s`で法線力、`[`で持ち上げ開始、HOLD後は`i/k`と`j/l`でXY、
`[/]`でZ、`q/e`でyawを操作します。Spaceは物体の速度指令を止めます。
`x`/Ctrl-Cでデモを終了します。終了は物体を台座に設置する操作ではありません。

## 中止・限界

- frame不一致、NaN、不正Quaternion、時刻の古い/未来の入力は拒否します。
- 接近中も保持中も、全台の位置・CoG・observer・flight_stateと物体姿勢の鮮度を確認します。
- 既定0.3秒の入力途絶/制御ループ停止、HOVER離脱、設定加速度上限超過で中止をラッチします。
  入力が回復しても自動再開しません。HOVERを続けている機体へXYZ速度0を送り、ACC指令を解除します。
  姿勢目標はその場で巻き戻しません。LAND/STOP等の機体へは停止指令も送らず、その処理を優先します。
- 押し付け解除で物体が落ちる可能性があります。中止は安全な物体設置を保証しません。
- プロセス内watchdogは、PC電源断、OS停止、通信断、FCの自己位置喪失には対処できません。
  別系統の操縦者介入・FC側watchdogが必要です。速度0も位置保持を保証しません。
- 実機の摩擦・サーボ応答・力推定バイアス・遅延を測定して段階的に検証してください。
  `configuration_verified`は操縦者確認用のインターロックで、自動安全認証ではありません。

## オフラインテスト

```bash
PYTHONDONTWRITEBYTECODE=1 python3 "$(rospack find bee)/test/test_real_grasp_demo.py"
```

ROS APIをmockし、入力/指令ゲートとMuJoCo互換性を検証します。実機性能の試験ではありません。
