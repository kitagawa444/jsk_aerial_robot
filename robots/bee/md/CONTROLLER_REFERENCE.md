# BeeControllerの共通指令追従（2026-11-16）

各Beeは上位から運動・力の目標を受け、既存のBeeControllerで追従します。
物体位置、法線、摩擦、把持フェーズ、接触中かどうかは下流の入力・判断に含めません。
`ReferenceTracker`はBeeControllerが所有する内部部品です。更新はBeeControllerの既存ループから
呼び出し、独立の制御timerやtrackerノードは起動しません。力計算の更新周期も既存の飛行制御周期に従います。
通常の`uav/nav`による位置・速度・加速度制御も、共通参照が動作していない間は使用できます。

| 入出力（機体名前空間の下） | 型 | 役割 |
|---|---|---|
| `command` | `bee/BeeCommand` | 上位の運動・姿勢・力の指令 |
| `controller/tracking_status` | `bee/ControllerStatus` | ready/active、校正済み、観測力、速度、最終加速度、fault |
| `controller/force_feedback` | `bee/ForceFeedback` | 力誤差、PI、LPF、積分、減衰、飽和状態 |
| `controller/calibrate_observer` | `std_msgs/Empty` | observerのゼロ点更新を1回要求 |
| `controller/observer_calibration` | `bee/ObserverCalibrationStatus` | latchedの校正状態・適用したゼロ点 |
| `controller/set_reference_mode` | `std_srvs/SetBool` | 参照受付の有効/無効、fault後の再開操作 |
| `controller/set_force_control_parameters` | `bee/SetForceControlParameters` | 力補償のゲイン・フィルターを一括更新するservice |
| `controller/force_control_parameters` | `bee/ForceControlParameters` | 現在の適用値をlatched配信 |

## 運動と力の指定

`BeeCommand.navigation`は既存のFlightNavです。並進の位置、速度、加速度を指定し、回転3軸はNO_NAVIGATIONとします。
`baselink_rpy`に世界基準の機体姿勢をradで指定し、Bee側でCoG目標姿勢に対する相対回転に変換します。通常の運動指令ではworld/local座標、
CoG/base_linkの指定を既存FlightNavへ引き継ぎます。
`enable_force_control=false`ならこの運動・姿勢指令を既存の飛行制御へ渡します。

`enable_force_control=true`の場合、world/CoG基準でXYはACCまたは位置・速度、ZはACCまたは位置指令です。位置指令では飛行制御の位置PIDに力補償の加速度を加えます。
`force`は**Beeから環境へ加える力**の3次元ベクトル[N]です。
observerは環境からBeeへ作用する力を返すため、力誤差は`force + observed_external_force`です。
機体が動くための加速度は`navigation.target_acc_*`に指定し、重力は既存飛行制御が補います。

```text
最終加速度 = 運動加速度 + 目標力 / 機体質量 + 力PI + 相対速度の減衰
```

`velocity`は減衰に使う目標速度、`velocity_point`はbase_link座標での制御点オフセットです。
ゼロならbase_link原点の速度を比較します。上位は任意の剛体固定点を指定でき、下流はその点が
接触点かどうかを判断しません。押付け中の法線運動はACC指令と力目標で指定します。

`acceleration_bounds=true`では、`acceleration_orientation`で指定した直交座標系に
`acceleration_min/max`の加速度範囲を適用します。同じ座標系が方向別Pゲインの軸になります。
範囲を指定しない場合はworld軸です。上下限を同じ値にすると、その成分を指定加速度に固定できます。
`reset_force_feedback`は上位からの明示的なPI/フィルタ状態リセットです。
これらは汎用の指令制限で、Bee側に物体の半径や法線の意味づけはありません。

`force_integral_enabled=false`は積分をゼロに戻します。力追従の開始/終了、停止、校正値の更新でも
状態をリセットします。PI＋速度減衰の合計には追加推力の予約枠、最終加速度には総推力L1上限を適用し、
飽和中は積分更新を止めます。推力制限が作用した場合は指定した加速度範囲・力目標を同時に満たせない
ことがあり、status/diagnosticで確認します。参照/状態の欠落や時刻異常は停止・faultとします。

## 校正

```bash
rostopic pub -1 /bee1/controller/calibrate_observer std_msgs/Empty '{}'
rostopic echo /bee1/controller/observer_calibration
```

力追従中でも受理します。接触状態や把持フェーズによる拒否はありません。
参照受付が無効でも校正でき、HOVERや上位参照は必要としません。
実行時点は上位・操作者が決めます。この操作は**その時の観測値をゼロ基準にする**ので、
荷重がかかっていれば荷重も基準に含みます。未知の荷重とobserverの偏りを分離する操作ではありません。

標準はROS時刻で2秒、異なるobserver時刻のサンプル20個以上です。基準を適用するまで旧値を保持し、
成功時に力フィルタとPIをリセットします。新しいcog姿勢/observerデータ、有限値、座標系、
時刻と収集区間を確認します。入力停止や時間停止/逆行、サンプル不足では失敗して旧値を保持します。
収集中の追加要求は拒否し、元の収集を継続します。これは収集処理の重複防止で、接触状態には依存しません。

状態は0=待機、1=収集中、2=成功、3=要求拒否、4=収集失敗。
`calibration_stamp`と`calibration_start_stamp`は最後に適用した基準の完了/開始時刻です。
body力/モーメントとworld力の基準を公開し、中央も同じ値を使用します。
生の`estimated_external_wrench`やobserver内部状態は変更しません。
参照受付ON/OFFでは基準を保持し、プロセス再起動では保持しません。
校正前はゼロオフセットで観測します。校正の完了を待つかどうかも上位の責任です。

## 設定と中央の役割

各Beeの設定は`config/controller/force_control.yaml`にまとめています。
`observer_profiles`で実機・simの推定設定を選び、`/beeN/controller/`へ展開します。
`reference_tracking`の値は`/beeN/controller/reference_tracking/`へ読み込みます。YAMLは起動時の初期値です。
`bringup.launch`、`multi_robot_bringup.launch`、`grasp_sim.launch`、`grasp_demo.launch`は
`force_control_config`で全体を差し替えられます。従来形式の追従設定だけを渡す場合は`reference_tracking_config`を使います。

- `force_kp: [0.4, 0.4, 0.4]`、`force_ki: 0.1`、積分上限`0.4 m/s²/軸`。
- `velocity_damping: 3.0`、`external_filter_tau: 0.2`秒。
- `force_filter_tau: 0.0`でPI出力LPFは無効。正の値ではPIだけを通し、FFと速度減衰は通しません。
- `thrust_limit: 32.0`N、`correction_reserve: 3.0`Nは運用上の上限です。
- 機体質量は`robot_description`のURDFから中央と下流で同じ関数により取得します。YAMLの`robot_mass`では変更しません。
  起動時の取得値は`controller/reference_tracking/model_mass`で確認できます。
  中央も各機の`robot_description`から質量を取得し、同じ質量であることを確認します。
  URDF未設定・無効・機体間の質量不一致では中央の起動を拒否します。中央の`model_mass`と
  `controller/robot_mass`は取得結果の診断値です。モデル変更後は中央と各Beeを再起動してください。
  KDLと同じくダミーrootの慣性を除外します。URDFに存在しない追加部品は自動算入されません。

中央の把持設定は`config/grasp/controller.yaml`に残します。
中央がQPの解を3次元力へ組み立て、接触基底を汎用の加速度範囲へ変換します。
押込み過ぎの判定と押戻し加速度も中央で計算し、必要な加速度範囲/状態リセットを送ります。
Preloadの既存の法線P経路は中央で加速度指令へ変換します。支持移行以降の力PIはBeeControllerで動きます。
今回、Preload/支持移行/Liftを共通QPにする別の改修や、Releaseの目的関数は変更していません。

把持のCalibrateフェーズに入ると、中央が各Beeの校正トピックへ要求し、要求後に収集を開始した
全機の結果がそろうまで次のフェーズへ進みません。これがタスク固有の判断であり、Bee側に
Calibrateフェーズの処理はありません。

参照受付は標準で有効ですが、新しい指令が来るまで通常の飛行操作を引き継ぎません。
無効化には`controller/set_reference_mode`へfalseを送り、faultから再開する場合はfalse→trueとします。
HOVER/SEPARATEDでの適用、LAND/STOPを上書きしないこと、入力の鮮度、推力上限は機体共通の条件です。

## 力補償パラメータのオンライン変更

中央・操作者は`/beeN/controller/set_force_control_parameters`を呼び、下流の再起動なしに
力Pゲイン3軸、Iゲイン、積分上限、速度減衰、PI出力LPF、観測LPFを変更できます。
下流はフェーズを解釈せず、要求された数値を適用します。フェーズ別の値や更新タイミングは上位が決めます。
今回、中央に新しいフェーズ別ゲイン切り替えは追加していません。

```bash
rosservice call /bee1/controller/set_force_control_parameters "
parameters:
  force_kp: {x: 0.4, y: 0.4, z: 0.4}
  force_ki: 0.1
  force_integral_limit: 0.4
  velocity_damping: 3.0
  force_filter_tau: 0.0
  external_filter_tau: 0.2
reset_force_feedback: false"

rostopic echo /bee1/controller/force_control_parameters
```

上の例は標準値です。`parameters`は差分ではなく全項目の置換です。
サービス応答の`success`を確認してください。`applied`には成功/拒否のどちらでも実際の適用値を返します。
有限かつ非負の値だけを受け付け、不正な項目があれば設定・制御状態を一切変更しません。
全項目は制御更新と同じmutex内で一括適用し、次回の制御計算から使用します。
観測LPF、PI出力LPFとも時定数0で無効です。Pゲインの軸は共通参照の加速度制限座標系、
制限を指定しない場合はworldです。Iと減衰は等方、積分上限はworld各軸の加速度単位です。

通常は積分の出力値と両LPFの状態を保持します。積分上限を下げると既存の積分値も制限し、
Iゲイン0では積分値をゼロにします。`reset_force_feedback=true`ではPI積分とPI出力LPFを
リセットしますが、observerのゼロ点と観測LPFは保持します。参照側の積分無効/リセット指令も引き続き有効です。
P/減衰の変更やLPFの無効化は出力を即時に変え得ます。自動的なゲイン補間は行わないため、
なめらかに切り替えたい場合は上位が中間値を順に送ります。変更の連続性・安定性をサービス自体で保証はしません。

参照が無効/未校正でも設定可能で、参照受付のON/OFFでは設定を保持します。
プロセス再起動時はYAMLへ戻ります。永続化する場合は初期設定も更新してください。
rosparamだけを書き換えても制御には反映されません。実行中の有効値は上記latchedトピックが基準です。
推力上限、追加補正の予約枠、機体質量、observerのスケールと入力監視条件はこのAPIの変更対象外です。
中央QPと共有する能力上限は起動設定で整合させます。

運搬試験、応答同定、実機用bag recorderは現在値トピックも記録します。
サービス応答はそのBeeでの適用確認で、複数Beeを同時刻に切り替える保証はありません。

## 移行と検証

`GraspMode`、独立`bee_grasp_tracker_node`、`grasp_tracker.launch`は廃止しました。
`grasp_mode`/`launch_grasp_tracker`のbringup引数、`grasp/set_mode`、`grasp/calibrate`は共通APIへ移行します。
中央と各Beeは新しいメッセージを含む版で再ビルド・再起動してください。
応答同定runnerは共通参照の `force` を直接変更します。

ワークスペース直下、ROS環境をsource済みの検証コマンド:

```bash
catkin build bee --no-deps -j4 -p1 --no-status
cmake --build build/bee --target reference_tracker_harness run_tests_bee_gtest_test_cooperative_grasp -j4
python3 src/jsk_aerial_robot/robots/bee/test/test_observer_calibration_ros.py \
  --native devel/.private/bee/lib/bee/reference_tracker_harness \
  --output /tmp/bee-reference-test
```

テストharnessは指令を私有の観察トピックへ返すだけで、飛行指令やモーター出力を送信しません。
この変更はローカル実装/シミュレーションの検証であり、実機への配布・実機動作確認とは区別します。

再利用できる検証入口は[FORCE_TESTS.md](FORCE_TESTS.md)、Livoxの移植範囲は[LIVOX.md](LIVOX.md)を参照してください。
