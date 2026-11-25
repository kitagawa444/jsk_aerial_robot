# BeeのLivox MID360 / FAST-LIO入口

gimbalrotorにあったLivox起動とナビゲーション設定を`aerial_robot_base`の共通部品へ移しました。
Beeとgimbalrotorは同じ部品を呼びます。gimbalrotorの既存launch名は互換入口として残しています。

- センサー共通部: `aerial_robot_base/launch/external_module/livox_mid360.launch`
- 点群→scan共通部: 同ディレクトリの`livox_scan.launch`
- move_base共通部: `livox_navigation.launch`、設定は`aerial_robot_base/config/navigation/livox/`
- Bee の単独起動には上記の共通 launch を使い、`robot_ns:=bee1` で機体を指定します。

## センサーだけ起動する入口

```bash
roslaunch aerial_robot_base livox_mid360.launch robot_ns:=bee1
```

これはLivoxドライバー、FAST-LIO、点群→scanだけを起動します。BeeController・Spinal接続・離陸を起動しません。
`robot_description`と`robot_state_publisher`も起動しません。機体とセンサー間のTFを使う場合は、
機体モデル側の`robot_state_publisher`が別途起動していることが前提です。
同じ機体でbringupのLivoxを有効にした場合は、この入口を重ねて起動しません。
実機のIP/シリアルやFAST-LIOのIMU–LiDAR外部パラメータは、手元のdriver/configと実機を照合する必要があります。
今回、実機へ接続して測定してはいません。

Beeのbringupから使う場合は`livox_enable:=true`を指定します。既定はfalseで従来のmocap構成を維持します。
Gazeboでは`simulation:=true real_machine:=false lidar_sim:=true`でモデルのLiDARとFAST-LIOを有効にします。
MuJoCoの今回の力試験にはLivoxデータ生成を実装していないため、MuJoCoではこの経路を起動しません。
`livox_mid360.launch simulation:=true`単独ではドライバーを起動せず、別途シミュレータの点群/IMUが必要です。

各機のtopicは`/beeN/livox/lidar`、`/beeN/livox/imu`、`/beeN/Odometry_precede`、`/beeN/livox/scan`。
ドライバーのframeは`beeN/lidar_imu`、FAST-LIOのprefixも`beeN`にそろえます。
点群→scanは1つだけ起動します。driver付属launchの副作用による二重起動を避けるため、共通部でdriverノードを直接定義しています。
FAST-LIOの更新によって引数/APIが変わった場合は、この接続を再検証してください。

## URDFとフレーム名

Beeの実機用・Gazebo用の展開済みURDFには、次の固定接続があります。

```text
root → base_link → lidar_imu
                → fc
```

ROSのTFには機体名が付き、例えば`bee1/base_link → bee1/lidar_imu`です。
一方、`sensor_plugin/vo/sensor_frame: lidar_imu`は、飛行制御内部のURDF運動学モデルの
リンク名を検索する設定なので、ここに`bee1/`は付けません。
`can not find lidar_imu in kinematics model`はこの内部モデルの検索エラーです。
ROS TFの名前が見つからないエラーとは区別し、読み込まれた`robot_description`を確認します。

現在の取付値はbase_linkからxyz=`[0.011, 0.02329, 0.07488]` m、rpy=`[0, 0, 0]`です。
URDFに`TODO: the true pos`が残っており、実機の取付位置・向きとの照合は未実施です。
リンクが存在しTFが取得できることと、実機の外部パラメータが正しいことは別の確認です。

2026-11-16、実際のbringupから生成されるURDFを実機/Gazebo×Livox有効/無効の4条件で検査しました。
加えて隔離ROS上の`robot_state_publisher`だけを動かし、実機モデルbee1とGazeboモデルbee2で
base_link/root/fcとlidar_imuの計6変換を確認しました。飛行制御・センサー実機は起動していません。

```bash
python3 robots/bee/test/test_livox_launch.py
```

これはlaunchの展開・namespace・driver有無・重複ノードと、展開後URDFのリンク接続・frameを調べる検査です。
Livox実機・Gazebo点群・FAST-LIOの測位精度の動作試験とは区別してください。
`roslaunch aerial_robot_base livox_navigation.launch robot_ns:=bee1` は飛行速度指令につながる従来のmove_base経路です。今回起動していません。

## 状態推定の設定と未検証事項

状態推定モード0とLivox起動は別の引数です。両方を選ぶ指定は次のとおりです。
これは起動構成の説明であり、この組合せの実機飛行を検証済みという意味ではありません。

```bash
roslaunch bee bringup.launch estimate_mode:=0 livox_enable:=true
```

`config/StateEstimation.yaml`の`estimate_mode`は、最上位では0/1/2のモード番号、
sensor_plugin以下では利用する推定系のビットマスクです。IMU=3（0と1）、mocap=6（1と2）、
vo1=1（0のみ）の組合せは整合しています。`sensor_plugin/vo`の共通値と`vo1`の個別値は
SensorBaseが両方読み込むため、名前の不一致ではありません。

| 経路 | 入力・設定 |
|---|---|
| 機体のIMU → Bee状態推定 | `/beeN/imu`、`spinal/Imu` |
| Livox内蔵IMU・点群 → FAST-LIO | `/beeN/livox/imu`、`/beeN/livox/lidar` |
| FAST-LIO → BeeのVOプラグイン | `/beeN/Odometry_precede`、位置＋速度、global速度 |
| 機体内部のセンサーフレーム | `lidar_imu` |

2026-11-16、実際のBeeモデルと状態推定プラグインだけを隔離ROSで動かし、合成IMUと
Odometryで検証しました。IMU校正待ち、mocapなしのXYZ初期化、20 cmの移動追従を確認しています。
実際のLiDAR点群からFAST-LIOが測位できることや、実機の飛行ゲインを確認した試験ではありません。

この監査では次の課題も確認しています。自律飛行の検証完了とは扱いません。

- IMU/VOの`unhealth_level`は未指定で、現在の既定値は0です。入力を止めると既定0.5秒で
  エラーログは出ますが、この状態推定経路の異常レベルは0のまま、XYZの有効フラグも残りました。
  既存navigationの異常レベル3による強制着陸には接続されていません。今回、自動着陸などの動作設定は変更していません。
- 現在利用しているFAST-LIOの`Odometry_precede`はIMUコールバックで予測値を配信します。
  点群更新と独立した経路なので、これの受信継続だけではLiDAR測位の更新を確認できません。
  点群およびLiDAR更新後の`Odometry`などを別に監視する必要があります。この点はソースを追った確認です。
- ドライバーの設定は接続先LiDAR `192.168.2.141`、受信側 `192.168.2.50` を指定しています。
  実機ネットワークとの一致、センサーの取付値、時刻・遅延・ノイズ設定の実測妥当性は未確認です。

FAST-LIOとドライバーは、この開発環境では別workspaceのパッケージを参照しています。
実機へ移す際は、それらの版と設定も含めて一致を確認する必要があります。

## シミュレータで機体の外観を確認する

`livox:=true` で Livox の取付部と本体を表示する。既定は `false`。既存の `lidar_sim:=true`（Gazebo のセンサー）または `livox_enable:=true`（実機ドライバー）でも、外観は既定で有効になる。MuJoCo の `livox` は表示用で、質量・慣性と点群の設定は独立している。

~~~bash
roslaunch bee bringup.launch real_machine:=false simulation:=true robot_id:=2 livox:=true headless:=false
roslaunch bee bringup.launch real_machine:=false simulation:=true mujoco:=true robot_id:=3 livox:=true headless:=false
roslaunch bee grasp_sim.launch robot_count:=3 livox:=true headless:=false
~~~

各行は個別の起動例。機体色は `config/body_colors.yaml` の RGBA（0～1）で指定する。標準は黄、ID 2 は白、ID 3 はネイビー。羽根の色は同じファイルの `propeller` で指定する。ID 1 は黄、ID 2 は赤、ID 3 とその他の機体は黒。
